#!/usr/bin/env python
#

import networkx as nx
import matplotlib.pyplot as plt

from network import Network
from directional_antenna import KNN, MST 
network = Network(number_of_nodes=40, relays=6, width=20.0, height=20.0)
network.beamsets()

print "DAKNN: %e" % KNN(network.copy(), K=3, beams=8)
print "DAMST: %e" % MST(network, beams=8)

network.draw()
plt.show()
