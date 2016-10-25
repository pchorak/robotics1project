# Probabilistic RoadMap path planning for the dobot
import numpy as np
import numpy.random as rng
import scipy.spatial.kdtree as kdt
import networkx as nx

import Simulation
import DobotModel

class Roadmap:
    """
    Probabalistic RoadMap

    Input: n - number of desired samples (in free space)

    Example:
        import Simulation,Roadmap
        pi = 3.141592
        prm = Roadmap.Roadmap(50)
        path = prm.get_path((pi/4,pi/3,0),(-pi/4,pi/3,0))
        prm.plot_path(path)

        Simulation.display(path[0])
    """

    #G - graph of configurations sampled from Qfree
    #tree - KD-tree of positions sampled from workspace 

    def __init__(self,n):
        self.G = nx.Graph()

        # Sample environment
        ps = self._sample_cs(n,DobotModel.limits)
#       ps = self._sample_ws(n,np.array([[0,300],[-200,200],[0,300]]))

        self.tree = kdt.KDTree(ps)

        # Connect samples
        for k in xrange(self.tree.n):
            self._connect(k,self.tree.data[k])

        # Skipping enhancement stage based on the assumption that Qfree >> Q!free

    def get_path(self,q0,qf):
        """
        Searches for a path in the PRM between configurations q0 and qf.
        Returns a list of configuration tuples describing the path or []
        if no path is found.
        """
        n0 = len(self.G.node)
        nf = n0 + 1

        # Add the start and end configs to G, so we can just search it
        self.G.add_node(n0,cfg=q0)
        self.G.add_node(nf,cfg=qf)
        for k in [n0,nf]:
            self._connect(k,DobotModel.forward_kinematics(self.G.node[k]['cfg']))

        if not nx.has_path(self.G,n0,nf):
            path = [] # could not find a path
        else:
            nodes = nx.dijkstra_path(self.G,n0,nf,'weight')
            path = [self.G.node[k]['cfg'] for k in nodes]

        # Remove the start and end configs so G remains consistent with tree
        self.G.remove_node(n0)
        self.G.remove_node(nf)

        return path

    def plot_path(path):
        """
        Plots points for all sampled positions of the end effector and draws lines
        along the path described by the configurations "path".
        """
        qs = [self.G.node[k]['cfg'] for k in range(len(self.G.node))]
        ps = np.array([DobotModel.forward_kinematics(q) for q in qs])
        # ps = [self.tree.data[k] for k in range(len(self.tree.data))]

        path = np.array([DobotModel.forward_kinematics(q) for q in path])

        fig = plt.gcf()
        ax = Axes3D(fig)
        for To in obstacles:
            ax.plot(To[[0,1,2,0],0],To[[0,1,2,0],1],To[[0,1,2,0],2],'b')
        ax.plot(path[:,0],path[:,1],path[:,2],'r')
        ax.plot(ps[:,0],ps[:,1],ps[:,2],'.g')
        plt.xlim([-50,350])
        plt.ylim([-200,200])
        plt.show()

    def _connect(self,k,p,knn=5):
        # Attempt to connect a node k (position p) with its k nearest neighbors (knn)
        (dists,inds) = self.tree.query(p,knn)
        for (d,i) in zip(dists,inds):
            if (k != i) and self._path_exists(k,i):
                self.G.add_edge(k,i,weight=d)

    def _path_exists(self,k0,k1):
        # Test whether there is a direct path between nodes k0 and k1 (no collisions)
        q0 = np.array(self.G.node[k0]['cfg'])
        q1 = np.array(self.G.node[k1]['cfg'])

        # Sample the path at ~10mm intervals based on dp/dq at the midpoint
        J = DobotModel.jacobian(tuple((q0+q1)/2.0))
        dp = np.dot(J,q1-q0)
        n = int(np.ceil(np.sqrt(np.dot(dp,np.transpose(dp)))/10.0))

        # Interpolate intermediate configurations
        qk = zip(np.linspace(q0[0],q1[0],n),np.linspace(q0[1],q1[1],n), \
            np.linspace(q0[2],q1[2]))

        # Test for collisions for each
        for q in qk:
            if Simulation.collision(q):
                return False

        return True

    def _sample_ws(self,n,bnds):
        # Sample the workspace until n valid points are found within the bounds
        ps = np.zeros((n,3))
        k = 0
        while (k < n):
            p = rng.rand(bnds.shape[0])*(bnds[:,1] - bnds[:,0]) + bnds[:,0]
            q = DobotModel.inverse_kinematics(p)
            if not any(np.isnan(q)) and not Simulation.collision(q):
                self.G.add_node(k,cfg=q)
                ps[k] = p
                k+=1
        return ps

    def _sample_cs(self,n,bnds):
        # Sample the configuration space until n valid points are found within the bounds
        ps = np.zeros((n,3))
        k = 0
        while (k < n):
            q = tuple(rng.rand(bnds.shape[0])*(bnds[:,1] - bnds[:,0]) + bnds[:,0])
            if DobotModel.valid_angles(q) and not Simulation.collision(q):
                self.G.add_node(k,cfg=q)
                ps[k] = DobotModel.forward_kinematics(q)
                k+=1
        return ps

