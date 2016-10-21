# Probabilistic RoadMap path planning for the dobot
import numpy as np
import numpy.random as rng
import scipy.spatial.kdtree as kdt
import networkx as nx

import model
import kinematics

class PRM:
    """
    Probabalistic RoadMap

    Input: n - number of desired samples (in free space)

    Example:
        import model,prm
        pi = 3.141592
        m = prm.PRM(50)
        path = m.get_path((pi/4,pi/3,0),(-pi/4,pi/3,0))
        model.display(path[0])

        qs = [m.G.node[k]['cfg'] for k in range(len(m.G.node))]
        model.plot_path(path,qs)
    """

    #G - graph of configurations sampled from Qfree
    #tree - KD-tree of positions sampled from workspace

    bnds = np.array([[-65,-5,-10],[65,85,95]])*np.pi/180.0
#   bnds = np.array([[0,-200,0],[300,200,300]])

    def __init__(self,n):
        self.G = nx.Graph()

        # Sample environment
        ps = self._sample_cs(n,self.bnds)
#       ps = self._sample_ws(n,self.bnds)

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
            self._connect(k,kinematics.forward(self.G.node[k]['cfg']))

        if not nx.has_path(self.G,n0,nf):
            path = [] # could not find a path
        else:
            nodes = nx.dijkstra_path(self.G,n0,nf,'weight')
            path = [self.G.node[k]['cfg'] for k in nodes]

        # Remove the start and end configs so G remains consistent with tree
        self.G.remove_node(n0)
        self.G.remove_node(nf)

        return path

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
        J = kinematics.jacobian(tuple((q0+q1)/2.0))
        dp = np.dot(J,q1-q0)
        n = int(np.ceil(np.sqrt(np.dot(dp,np.transpose(dp)))/10.0))

        # Interpolate intermediate configurations
        qk = zip(np.linspace(q0[0],q1[0],n),np.linspace(q0[1],q1[1],n), \
            np.linspace(q0[2],q1[2]))

        # Test for collisions for each
        for q in qk:
            if model.collision(q):
                return False

        return True

    def _sample_ws(self,n,bnds):
        # Sample the workspace until n valid points are found within the bounds
        ps = np.zeros((n,3))
        k = 0
        while (k < n):
            p = rng.rand(bnds.shape[1])*(bnds[1] - bnds[0]) + bnds[0]
            q = kinematics.inverse(p)
            if not np.isnan(psi) and not model.collision(q):
                self.G.add_node(k,cfg=q)
                ps[k] = p
                k+=1
        return ps

    def _sample_cs(self,n,bnds):
        # Sample the configuration space until n valid points are found within the bounds
        ps = np.zeros((n,3))
        k = 0
        while (k < n):
            q = tuple(rng.rand(bnds.shape[1])*(bnds[1] - bnds[0]) + bnds[0])
            if not model.collision(q):
                self.G.add_node(k,cfg=q)
                ps[k] = kinematics.forward(q)
                k+=1
        return ps

