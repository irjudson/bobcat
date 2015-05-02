#!/usr/bin/env python
#

import networkx as nx
import matplotlib.pyplot as plt

from network import Network
from beam_scheduling import greedy1, greedy2, optimal

network = Network(number_of_nodes=30, relays=6, radial=True, width=20.0, height=20.0)
network.beamsets()
print("BSRAP Greedy 1: ", greedy1(network))
print("BSRAP Greedy 2: ", greedy2(network))

print("BSRAP Optimal: ", optimal(network))

network.draw()
plt.show()
