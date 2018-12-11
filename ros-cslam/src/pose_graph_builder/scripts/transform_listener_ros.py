#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header, Time
from geometry_msgs.msg import *
import duckietown_cslam.duckietownGraphBuilder.duckietown_graph_builder as dGB
import g2o
import numpy as np
import geometry as g
import yaml
import tf_conversions
import tf2_ros
import threading


class TransformListener():
    """Listens for the transforms published by the acquisition node, associates
       each object in the map to an ID, builds an internal pose graph and
       broadcasts the (optimized) tree of transforms, to be e.g. visualized with
       duckietown-visualization.

       Attributes:
           pose_graph: Pose graph
           old_odometry_stamps: Stores, for each ID, the time stamp of the last
                                odometry message read for the object with that
                                ID.
           id_map: Contains all April tags, mapped to their ID and type based on
                   the database read.
           last_callback: Stores the time when the last callback was started.
           optim_period: Sets the time (in seconds) that should pass before a
                         new optimization step is performed.
           optim_period_counter: Stores the time that one should look at to
                                 decide whether to perform a new optimization
                                 step.
    """

    def __init__(self):
        self.pose_graph = None
        self.old_odometry_stamps = {}
        self.id_map = {}
        self.last_callback = rospy.get_time()
        self.optim_period = 0.5
        self.optim_period_counter = -10.0
        # self.lock = threading.Lock()

    def initialize_id_map(self):
        """ Loads April tags into the ID map, assigning each tag in the database
            its ID and its type (e.g. TrafficSign, Localization, etc.).
        """
        global id_map
        config_folder = rospy.get_param("config_folder")
        aprilstagDB = "%s/%s" % (config_folder, "apriltagsDB.yaml")
        # Read YAML file.
        with open(aprilstagDB, 'r') as stream:
            try:
                complete_dict = yaml.safe_load(stream)
                for myobject in complete_dict:
                    tag_id = myobject["tag_id"]
                    mytype = myobject['tag_type']
                    self.id_map[str(tag_id)] = mytype
            except yaml.YAMLError as exc:
                print(exc)

    def find_vertex_name(self, id):
        """ Returns the format ID of an object in the ID map based on its type.

            Args:
                id: Original ID of the object in the ID map.

            Returns:
                ID of the objected, formatted by adding "duckie_" or "apriltag_"
                to it, based on its type.
        """
        if (self.id_map[id] == "Vehicle"):
            id = "duckie_%s" % id
        else:
            id = "apriltag_%s" % id

        return id

    def handle_odometry_message(self, id, transform, time_stamp):
        """Processes an odometry message, adding an edge to the graph and
           keeping track of the last time stamp before each new odometry message
           (needed to handle edges in the pose graph and connect nodes).

           Args:
               id: ID of the object sending the odometry message.
               transform: Transform contained in the ROS message.
               time_stamp: Timestamp associated to the ROS message.
        """
        # By default assign the time stamp of the last odometry message to be at
        # time 0. This is needed when the actual first odometry message for a
        # certain ID is read.
        old_time_stamp = 0

        # Get the time stamp of the previous odometry message and update the
        # time stamp of the last odometry message with the current timestamp.
        if (id in self.old_odometry_stamps):
            old_time_stamp = self.old_odometry_stamps[id]
        self.old_odometry_stamps[id] = time_stamp

        # Add edge to the graph.
        self.pose_graph.add_edge(id, id, transform, time_stamp, old_time_stamp)

    def handle_watchtower_message(self, id0, id1, transform, time_stamp):
        """Processes a message containing the pose of an object seen by a
           watchtower and adds an edge to the graph. If the object seen is a
           Duckiebot, adjusts the pose accordingly.

           Args:
               id0: ID of the object (watchtower) that sees the April tag of the
                    other object.
               id1: ID of the object whose April tag is seen by the watchtower.
               transform: Transform contained in the ROS message.
               time_stamp: Timestamp associated to the ROS message.
        """
        # Get type of the object seen.
        type_of_object_seen = id1.split("_")[0]
        if (type_of_object_seen == "duckie"):
            # In case of Duckiebot the pose needs to be adjusted to take into
            # account the pose of the April tag w.r.t. the base frame of the
            # Duckiebot.
            t = [0.0, 0.0, 0.1]
            z_angle = 90
            z_angle = np.deg2rad(z_angle)
            x_angle = np.deg2rad(180)
            R_z = g.rotation_from_axis_angle(np.array([0, 0, 1]), z_angle)
            R_x = g.rotation_from_axis_angle(np.array([1, 0, 0]), x_angle)
            R = np.matmul(R_x, R_z)
            H_apriltag_to_base = g2o.Isometry3d(R, t)
            transform = transform * H_apriltag_to_base

        # Add edge to the graph.
        self.pose_graph.add_edge(id0, id1, transform, time_stamp)

    def handle_duckiebot_message(self, id0, id1, transform, time_stamp):
        """Processes a message containing the pose of an object seen by a
           Duckiebot and adds an edge to the graph. Note: we assume that a
           Duckiebot cannot see the April tag of another Duckiebot, so no
           adjustment based on the object seen is needed.

           Args:
               id0: ID of the object (Duckiebot) that sees the April tag of the
                    other object.
               id1: ID of the object whose April tag is seen by the Duckiebot.
               transform: Transform contained in the ROS message.
               time_stamp: Timestamp associated to the ROS message.
        """
        # Get type of the object that sees the other object, for a sanity check.
        type_of_object_seeing = id0.split("_")[0]
        if (type_of_object_seeing == "duckie"):
            # The pose needs to be adjusted to take into account the relative
            # pose of the camera on the Duckiebot w.r.t. to the base frame of
            # the Duckiebot.
            t = [0.1, 0.0, 0.1]
            # This angle is an estimate of the angle by which the plastic
            # support that holds the camera is tilted.
            y_angle = 105
            z_angle = -90
            y_angle = np.deg2rad(y_angle)
            z_angle = np.deg2rad(z_angle)
            R_y = g.rotation_from_axis_angle(np.array([0, 1, 0]), y_angle)

            R_z = g.rotation_from_axis_angle(np.array([0, 0, 1]), z_angle)
            R = np.matmul(R_y, R_z)
            H_base_to_camera = g2o.Isometry3d(R, t)
            transform = H_base_to_camera * transform
        else:
            print("This should not be here!")

        # Add edge to the graph.
        self.pose_graph.add_edge(id0, id1, transform, time_stamp)

    def filter_name(self, id):
        """ Converts the frame IDs of the objects in the ROS messages (e.g.,
            Duckiebots, watchtowers, etc.) to the format <type>_<tag_id>, where
            <type> should be one of the types defined in DuckietownGraphBuilder
            (e.g. "duckie", "watchtower", "apriltag") and <tag_id> is the ID of
            the April tag of the object in the ID map.

            Args:
                id: Frame ID in the ROS message, to be converted.

            Returns:
                Converted frame ID.
        """
        if (id == "donaldthegreat"):
            id = "duckie_88"

        elif (id.startswith("watchtower")):
            id = "watchtower_%d" % int(id.strip("watchtower"))

        elif (len(id.split("_")) == 1):
            id = self.find_vertex_name(id)

        return id

    def callback(self, data):
        """ ROS callback.
        """
        # Update time of last callback and time counter for the optimization.
        start_time = rospy.get_time()
        self.optim_period_counter += start_time - self.last_callback
        self.last_callback = start_time
        # Get frame IDs of the objects to which the ROS messages are referred.
        id0 = data.header.frame_id
        id1 = data.child_frame_id
        # Convert the frame IDs to the right format.
        id0 = self.filter_name(id0)
        id1 = self.filter_name(id1)

        # For debug. TODO: remove.
        if (id0 == "watchtower_5"):
            # self.lock.release()
            return 0

        # Ignore messages from one watchtower to another watchtower (i.e.,
        # odometry messages between watchtowers). TODO: check if we can avoid
        # sending these messages.
        is_from_watchtower = False
        if (id0.startswith("watchtower")):
            is_from_watchtower = True
            if (id1.startswith("watchtower")):
                # print(data)
                # self.lock.release()
                return 0

        # Create translation vector.
        t = [
            data.transform.translation.x, data.transform.translation.y,
            data.transform.translation.z
        ]

        # Create rotation matrix. NOTE: in Pygeometry, quaternion is
        # the (w, x, y, z) form.
        q = [
            data.transform.rotation.w, data.transform.rotation.x,
            data.transform.rotation.y, data.transform.rotation.z
        ]
        M = g.rotations.rotation_from_quaternion(np.array(q))

        # Verify that the rotation is a proper rotation.
        det = np.linalg.det(M)
        if (det < 0):
            print("det is %f" % det)

        # Obtain complete transform and use it to add vertices in the graph.
        transform = g2o.Isometry3d(M, t)
        time = Time(data.header.stamp)
        time_stamp = time.data.secs + time.data.nsecs * 10**(-9)

        if (id1 == id0):
            # Same ID: odometry message, e.g. the same Duckiebot sending
            # odometry information at different instances in time.
            self.handle_odometry_message(id1, transform, time_stamp)
        elif (is_from_watchtower):
            # Tag detected by a watchtower.
            self.handle_watchtower_message(id0, id1, transform, time_stamp)
        else:
            # Tag detected by a Duckiebot.
            self.handle_duckiebot_message(id0, id1, transform, time_stamp)

        # If enough time has passed since the last optimization, perform a new
        # one and reset the optimization counter.
        if (self.optim_period_counter > self.optim_period):
            self.pose_graph.optimize(
                10,
                save_result=True,
                verbose=True,
                output_name="/tmp/test2.g2o")
            self.optim_period_counter = 0

            # Broadcast tree of transforms with TF.
            pose_dict = self.pose_graph.get_all_poses()
            for node_type, node_list in pose_dict.iteritems():
                for node_id, node_pose in node_list.iteritems():
                    self.tfbroadcast(node_type, node_id, node_pose)

        end_time = rospy.get_time()
        diff_time = end_time - start_time
        self.last_callback = rospy.get_time()

    def tfbroadcast(self, node_type, node_id, node_pose):
        """ Brodcasts a node in the tree of transforms with TF.

            Args:
                node_type: Type of the node. Can be any of the types defined in
                           the class DuckietownGraphBuilder.
                node_id: ID of the node.
                node_pose: Pose of the node.
        """
        # Create broadcaster and transform.
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()

        # Set frame ID. TODO: change it depending on the node type.
        if (node_type == "duckie"):
            t.header.frame_id = "map"
        else:
            t.header.frame_id = "map"
        # Set child frame ID.
        t.child_frame_id = "%s_%s" % (node_type, node_id)
        # Set transform:
        # - Create translation vector.
        t.transform.translation.x = node_pose.t[0]
        t.transform.translation.y = node_pose.t[1]
        t.transform.translation.z = node_pose.t[2]
        # - Create rotation matrix.
        #   Verify that the rotation is a proper rotation.
        det = np.linalg.det(node_pose.R)
        if (det < 0):
            print("after optim : det = %f" % det)
        #   NOTE: in Pygeometry, quaternion is the (w, x, y, z) form.
        q = g.rotations.quaternion_from_rotation(node_pose.R)
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        # Send the transform.
        br.sendTransform(t)

    def listen(self):
        """Initializes the graph based on the floor map and initializes the ID
           map. Then starts listening to the poses and odometry topics published
           by the acquistion node.
        """
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        initial_floor_april_tags = "%s/%s" % (rospy.get_param("config_folder"),
                                              "robotarium1.yaml")
        # Build graph based on floor map.
        self.pose_graph = dGB.DuckietownGraphBuilder(
            initial_floor_april_tags=initial_floor_april_tags)
        # Initialize ID map.
        self.initialize_id_map()
        # Subscribe to topics.
        rospy.Subscriber("/poses_acquisition/poses", TransformStamped,
                         self.callback)
        rospy.Subscriber("/poses_acquisition/odometry", TransformStamped,
                         self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


def main():
    rospy.init_node('listener', anonymous=True)

    tflistener = TransformListener()
    tflistener.listen()


if __name__ == '__main__':
    main()
