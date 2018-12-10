import numpy as np
import g2o
import geometry as g


class g2oGraphBuilder():

    def __init__(self):
        self.optimizer = g2o.SparseOptimizer()
        self.solver = g2o.BlockSolverX(g2o.LinearSolverDenseX())
        self.algorithm = g2o.OptimizationAlgorithmLevenberg(self.solver)
        self.optimizer.set_algorithm(self.algorithm)
        self.already_initialized = False
        self.last_lost = 0

    def add_vertex(self, vertex_id, vertexPose, fixed=False):
        # vertexPose has to be Isometry3D
        vc = g2o.VertexSE3()
        vc.set_id(vertex_id)
        vc.set_estimate(vertexPose)

        if fixed:
            vc.set_fixed(True)
        self.optimizer.add_vertex(vc)

    def add_edge(self, vertex0Id, vertex1Id, measure):
        '''
        Helper function to add an edge
        vertex 0 and 1 are valid IDs of vertex inside optimizer.
        measure is a Isometry3d that map the transform from vertex0 to vertex1
        '''
        # print(optimizer.vertices())
        if vertex0Id in self.optimizer.vertices():
            if vertex1Id not in self.optimizer.vertices():
                vc1 = g2o.VertexSE3()
                vc1.set_id(vertex1Id)
                vc1.set_estimate(self.optimizer.vertex(
                    vertex0Id).estimate() * measure)
                vc1.set_fixed(False)
                self.optimizer.add_vertex(vc1)

            edge = g2o.EdgeSE3()
            edge.set_vertex(0, self.optimizer.vertex(vertex0Id))
            edge.set_vertex(1, self.optimizer.vertex(vertex1Id))
            edge.set_measurement(measure)
            # r = abs(g.rotation_from_axis_angle(
            #     np.array([1, 0, 0]), np.deg2rad(2)))
            # ligne1 = np.concatenate((r, r), axis=1)
            # ligne2 = np.concatenate((r, r), axis=1)
            # r_final = np.concatenate((ligne1, ligne2), axis=0)

            # edge.set_information(r_final)

            finished = self.optimizer.add_edge(edge)
            if(not finished):
                print("Adding edge in g2o is not finished")

        else:
            if vertex1Id in self.optimizer.vertices():
                vc0 = g2o.VertexSE3()
                vc0.set_id(vertex0Id)
                vc0.set_estimate(self.optimizer.vertex(
                    vertex1Id).estimate() * measure.inverse())
                vc0.set_fixed(False)
                self.optimizer.add_vertex(vc0)
                edge = g2o.EdgeSE3()
                edge.set_vertex(0, self.optimizer.vertex(vertex0Id))
                edge.set_vertex(1, self.optimizer.vertex(vertex1Id))
                edge.set_measurement(measure)

                # edge.set_information(np.eye(6) * 2)
                finished = self.optimizer.add_edge(edge)

            else:
                vc0 = g2o.VertexSE3()
                vc0.set_id(vertex0Id)
                vc0.set_fixed(False)
                self.optimizer.add_vertex(vc0)
                self.add_edge(vertex0Id, vertex1Id, measure)

    def vertices_and_edges(self):
        return (self.optimizer.vertices(), self.optimizer.edges())

    def vertex_pose(self, vertexId):
        if(vertexId not in self.optimizer.vertices()):
            print("Vertex %i is not in the g2o graph" % vertexId)
            if(self.last_lost != 0 and self.last_lost in self.optimizer.vertices()):
                print("Vertex %i wasn't but is now in g2o graph" %
                      self.last_lost)
            elif(self.last_lost != 0):
                print("Vertex %i wasn't and still isn't in g2o graph" %
                      self.last_lost)
            self.last_lost = vertexId
        return (self.optimizer.vertex(vertexId).estimate())

    def remove_vertex(self, vertexId):
        self.optimizer.remove_vertex(vertexId)

        # optimizing
    def optimize(self, number_of_steps, verbose=True, save_result=True, output_name="output.g2o"):
        # if(not self.already_initialized):
        self.optimizer.set_verbose(verbose)

        self.optimizer.initialize_optimization()
        # self.already_initialized = True
        self.optimizer.compute_active_errors()
        if(verbose):
            print('Optimization:')
            print('Initial chi2 = %f' % self.optimizer.chi2())

        self.optimizer.optimize(number_of_steps)
        if(save_result):
            self.optimizer.save(output_name)
