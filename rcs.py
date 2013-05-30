#!/usr/bin/env python
#

import random

import networkx as nx
import matplotlib.pyplot as plt

from network import Network
from joint_routing_channel_selection import select_channels, select_channels_greedy, rcs_path, vertices_for_path

ITER = 1
network = Network(number_of_nodes=10, width=50.0, height=50.0)
network.initialize_edges()

# Create a set of paths
paths = list()
for i in range(ITER):
	src = None
	dst = None
	while src == dst:
		src = random.choice(network.nodes())
		dst = random.choice(network.nodes())
	paths.append((src, dst))

for path in paths:
    src, dst = path
    print "\nSource: %d Destination: %d" % (src, dst)
    dijkstra_shortest_paths = nx.single_source_dijkstra_path(network.copy(), src, weight='distance')
    dijkstra_shortest_path = dijkstra_shortest_paths[dst]
    dsp_edges = zip(dijkstra_shortest_path[0:], dijkstra_shortest_path[1:])
    print "\n\tDijkstra\n\t\tPath: %s" % dijkstra_shortest_path
    dijkstra_throughput = select_channels(network, dsp_edges)
    print "\t\tThroughput = %e" % dijkstra_throughput
    dijkstra_greedy_throughput = select_channels_greedy(network, dsp_edges)
    print "\t\tThroughput Greedy = %e" % dijkstra_greedy_throughput
    prim_tree = nx.prim_mst(network, weight='bottleneck_weight')
    prim_path = nx.shortest_path(prim_tree, src, dst)
    prim_edges = zip(prim_path[0:], prim_path[1:])
    print "\tPrim\n\t\tPath: %s" % prim_path
    prim_throughput = select_channels(network, prim_edges)
    print "\t\tThroughput = %e" % prim_throughput
    prim_greedy_throughput = select_channels_greedy(network, prim_edges)
    print "\t\tThroughput Greedy = %e" % prim_greedy_throughput
    rcs_path = rcs_path(network, src, dst)
    print "\tRCS"
    if rcs_path is not None:
        print "\t\tPath: %s" % list(vertices_for_path(rcs_path.path))
        rcs_throughput = select_channels(network, rcs_path.path)
        print "\t\tThroughput: %e" % rcs_throughput
        rcs_greedy_throughput = select_channels_greedy(network, rcs_path.path)
        print "\t\tThroughput Greedy: %e" % rcs_greedy_throughput
    else:
        print "\t\tPath: Not found."

# network.draw()
# plt.show()