#
#  Directional Antenna Algorithms
#  (c) 2013 Ivan R. Judson / Montana State University
#

import networkx

def KNN(network, K=3, beams=8):
    """ Directional Antenna Algorithm """
    # Neighbor dictionary for easy lookup later
    neighbors = dict()

    # Find nearest K_nearest_neighbors
    for n in network.nodes():
        node = network.node[n]
        nlist = [(neighbor, network.distance(n, neighbor)) for neighbor in networkx.all_neighbors(network, n)]
        neighbors[n] = [m[0] for m in sorted(nlist, key=lambda item: item[1])][0:K]

        # Set active beams / edges
        for neighbor in neighbors[n]:
            node['active_beams'].add(network.beam_index(n, neighbor, beams))

        # Remove unused edges
        to_remove = set(networkx.all_neighbors(network, n)) - set(neighbors[n])
        for m in to_remove:
            network.remove_edge(n,m)

    # Get Total Weight and return it
    return network.update_throughput_for_nodes()

def MST(network, beams=8):
    """ Directional Antenna Algorithm """
    # Get the MST
    T = networkx.minimum_spanning_tree(network, 'distance')

    # Set active beams
    for n in T.nodes():
        node = network.node[n]
        for neighbor in networkx.all_neighbors(network, n):
            node['active_beams'].add(network.beam_index(n, neighbor, beams))

    # Get Total Weight and return it
    return network.update_throughput_for_nodes()
