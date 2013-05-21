#!/usr/bin/env python
#

import random

import networkx as nx
import matplotlib.pyplot as plt

from network import Network
from joint_routing_channel_selection import select_channels, select_channels_greedy, rcs_path

ITER = 1
network = Network(number_of_nodes=25, width=50.0, height=50.0)
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
    print "Source: %d Destination: %d" % (src, dst)
    dijkstra_shortest_paths = nx.single_source_dijkstra_path(network.copy(), src, weight='distance')
    dijkstra_shortest_path = dijkstra_shortest_paths[dst]
    dsp_edges = zip(dijkstra_shortest_path[0:], dijkstra_shortest_path[1:])
    print "Dijkstra Path: %s" % dijkstra_shortest_path
    dijkstra_throughput = select_channels(network, dsp_edges)
    print "Dijkstra Throughput = %e" % dijkstra_throughput
    dijkstra_greedy_throughput = select_channels_greedy(network, dsp_edges)
#    print "Dijkstra Greedy Throughput = %e" % dijkstra_greedy_throughput
    prim_tree = nx.prim_mst(network, weight='bottleneck_weight')
    prim_path = nx.shortest_path(prim_tree, src, dst)
    prim_edges = zip(prim_path[0:], prim_path[1:])
    print "Prim Path: %s" % prim_path
    prim_throughput = select_channels(network, prim_edges)
    print "Prim Throughput = %e" % prim_throughput
#    prim_greedy_throughput = select_channels_greedy(network, prim_path)
#    print "Prim Greedy Throughput = %e" % prim_greedy_throughput#
#    rcs_path = network.get_rcs_path(src, dst)
#    print "RCS Path: %s" % rcs_path
#    rcs_throughput = select_channels(network, rcs_path)
#    print "RCS Throughput: %e" % rcs_throughput
#    rcs_greedy_throughput = select_channels_greedy(network, rcs_path)
#    print "RCS Greedy Throughput: %e" % rcs_greedy_throughput

network.draw()
plt.show()