# write tests for bfs
import pytest
import numpy as np
from mst import Graph
from sklearn.metrics import pairwise_distances


def check_mst(adj_mat: np.ndarray, 
              mst: np.ndarray, 
              expected_weight: int, 
              allowed_error: float = 0.0001):
    """ Helper function to check the correctness of the adjacency matrix encoding an MST.
        Note that because the MST of a graph is not guaranteed to be unique, we cannot 
        simply check for equality against a known MST of a graph. 
        Arguments:
            adj_mat: Adjacency matrix of full graph
            mst: Adjacency matrix of proposed minimum spanning tree
            expected_weight: weight of the minimum spanning tree of the full graph
            allowed_error: Allowed difference between proposed MST weight and `expected_weight`
        TODO: 
            Add additional assertions to ensure the correctness of your MST implementation
        For example, how many edges should a minimum spanning tree have? Are minimum spanning trees
        always connected? What else can you think of?
    """
    def approx_equal(a, b):
        # approximately equal, given a margin of error
        return abs(a - b) < allowed_error

    total = 0
    print(mst.shape[0])
    print(adj_mat)
    for i in range(mst.shape[0]):
        for j in range(i+1):
            total += mst[i, j]
    print(total)
    print(mst)
    # checks that the total weight of the generated mst is approx equal to expected_weight
    assert approx_equal(total, expected_weight), 'Proposed MST has incorrect expected weight'

    # EDGE CHECK; mst must have V-1 edges, where V is number vertices.
    nodes_mst = adj_mat.shape[0] # how many nodes?
    mst_edges = nodes_mst - 1 
    # count non_zero for each non zero is an edge
    non_zero = np.count_nonzero(mst)
    # divide count by 2 bc symmetric
    edges = non_zero/2 
    assert edges == mst_edges, 'Proposed MST has incorrect number of edges'

    # SYMMETRY CHECK; recall matrix is symm. if A = A ^ T
    # both mst and adj_mat are symm
    assert np.array_equal(mst, mst.T), 'Proposed MST is not symmetric'


def test_mst_small():
    """ Unit test for the construction of a minimum spanning tree on a small graph """
    file_path = './data/small.csv'
    g = Graph(file_path)
    g.construct_mst()
    check_mst(g.adj_mat, g.mst, 8)


def test_mst_single_cell_data():
    """ Unit test for the construction of a minimum spanning tree using 
    single cell data, taken from the Slingshot R package 
    (https://bioconductor.org/packages/release/bioc/html/slingshot.html)
    """
    file_path = './data/slingshot_example.txt'
    # load coordinates of single cells in low-dimensional subspace
    coords = np.loadtxt(file_path)
    # compute pairwise distances for all 140 cells to form an undirected weighted graph
    dist_mat = pairwise_distances(coords)
    g = Graph(dist_mat)
    g.construct_mst()
    check_mst(g.adj_mat, g.mst, 57.263561605571695)


def test_mst_student():
    """ Extra test given the following file matrix"""
    # my file is:
    #[0 2 3]
    #[2 0 4]
    #[3 4 0]
    #
    # mst:
    #[0 2 3]
    #[2 0 0]
    #[3 0 0 ]

    file = 'data/my_test.csv'
    g = Graph(file)
    g.construct_mst()

    check_mst(g.adj_mat, g.mst, 5)
