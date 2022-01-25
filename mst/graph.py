import numpy as np
import heapq
from typing import Union

class Graph:
    def __init__(self, adjacency_mat: Union[np.ndarray, str]):
        """ Unlike project 2, this Graph class takes an adjacency matrix as input. `adjacency_mat` 
        can either be a 2D numpy array of floats or the path to a CSV file containing a 2D numpy array of floats.

        In this project, we will assume `adjacency_mat` corresponds to the adjacency matrix of an undirected graph
        """
        if type(adjacency_mat) == str:
            self.adj_mat = self._load_adjacency_matrix_from_csv(adjacency_mat)
        elif type(adjacency_mat) == np.ndarray:
            self.adj_mat = adjacency_mat
        else: 
            raise TypeError('Input must be a valid path or an adjacency matrix')
        self.mst = None

    def _load_adjacency_matrix_from_csv(self, path: str) -> np.ndarray:
        with open(path) as f:
            return np.loadtxt(f, delimiter=',')

    def construct_mst(self):
        """ Given `self.adj_mat`, the adjacency matrix of a connected undirected graph, implement Prim's 
        algorithm to construct an adjacency matrix encoding the minimum spanning tree of `self.adj_mat`. 
            
        `self.adj_mat` is a 2D numpy array of floats. 
        Note that because we assume our input graph is undirected, `self.adj_mat` is symmetric. 
        Row i and column j represents the edge weight between vertex i and vertex j. An edge weight of zero indicates that no edge exists. 
        
        """

        # empty mst of same shape, will fill throughout 
        self.mst = np.zeros_like(self.adj_mat)

        # store total available nodes
        total_nodes = self.adj_mat.shape[0]

        # start at random node, I choose 0th  
        start_node = 0
        # already visited: start_node
        visited = [start_node]

        # log adj. nodes and dists in 1st row of self.adj 
        # here: (distance, from_start_node, to_end_node)
        logged_edges = [(dist, start_node, node) for node, dist in enumerate(self.adj_mat[0]) if dist != 0]
        # heapify: lightest weight first
        heapq.heapify(logged_edges)


        # while we have not visited everyone yet
        while len(visited) != total_nodes:
            # popping tuple: dist, from_start_node, to_end_node
            lightest, start, end = heapq.heappop(logged_edges)

            # if we have not visited the end_node
            if end not in visited:
                # fill destination [start][end] with dist
                # note symmetry
                self.mst[start][end] = lightest 
                self.mst[end][start] = lightest

                # we've visited someone new, and it was cheap
                visited.append(end)

                # let's checkout the hosts adj. neighbors
                # iterate through all possible neighbors (total_nodes), cols 
                for new_node in range(total_nodes):
                    # push to heap (logged edges) its adj. neighbors, following tuple pattern
                    # we'll heapify once we pop
                    # note that (self.adj_mat[end, new_node], end, node)
                    # is (the_new_distance, from_new_start_node, to_adjacent_neighbor)
                    heapq.heappush(logged_edges, (self.adj_mat[end, new_node], end, new_node))

                # hmm, how do i avoid zeros??? aka no edges 
