#
# Base Network Implementation
#

import sys
import math
import time
import random
import pprint
from collections import OrderedDict, defaultdict

import networkx

import matplotlib.pyplot as plt

verbose = False

def frequency(f_in_GHz):
    frequency = f_in_GHz * math.pow(10,9)
    return frequency

speed_of_light = 3.0 * math.pow(10,8)
wavelength = { 
    700 : speed_of_light / frequency(0.7),
    900 : speed_of_light / frequency(0.9),
    2.4 : speed_of_light / frequency(2.4),
    5.8 : speed_of_light / frequency(5.8)
}

signal_range = {
    700 : 30.8,
    2.4 : 9.0,
    5.8 : 3.6
}

class Network(networkx.graph.Graph):
    """
    Network class. Base of all simulation parts.
    """

    FREQUENCIES = ( 700, 2.4, 5.8 )

    def __init__(self, seed=None, width=20.0, height=20.0, 
                 number_of_nodes=10, relays=2, radial=False, 
                 sectors=8, theta=30.0, meanq=40000.0, slot_length=0.001, 
                 channels=4, channel_probability=0.3):
        """
        Create a network that spans a given size and number of nodes.
        """
        if not seed:
            seed = int(time.time()*1000000)

        # Super initialization 
        G = networkx.complete_graph(number_of_nodes) 

        if not radial:
            networkx.graph.Graph.__init__(self, data=G, width=width,
                                          height=height, seed=seed)
        else:
            networkx.graph.Graph.__init__(self, width=width, height=height,
                                          seed=seed)
            self.add_nodes_from(G.nodes())
 
        # Set attributes
        self.width = width
        self.height = height
        self.number_of_nodes = number_of_nodes
        self.seed = seed
        self.sectors = sectors
        self.theta = theta
        self.meanq = meanq
        self.slot_length=slot_length
        self.channels = channels
        self.channel_probability = channel_probability
        self.radial = radial
        self.position = dict()
        self.relays = set()
        self.subscribers = set()
        self.interference = defaultdict(lambda: defaultdict(dict))
        self.beamset = defaultdict(lambda: defaultdict(dict))

        # Distribute the nodes randomly throughout the space
        for node in self:
            self.node[node]['type'] = 2
            self.node[node]['sectors'] = sectors
            self.node[node]['transmitter_gain'] = math.pow(10, 2.0 / 10)
            self.node[node]['receiver_gain'] = math.pow(10, 2.0 / 10)
            self.node[node]['queue_length'] = random.random() * 2.0 * self.meanq
            self.node[node]['active_beams'] = set()
            self.node[node]['in_throughput'] = 0.0
            self.node[node]['out_throughput'] = 0.0
            self.node[node]['throughput'] = 0.0
            if radial:
                radius = random.random() * self.width
                theta = math.radians(random.random() * 360.0)
                if node == 0:
                    location_x = 0.0
                    location_y = 0.0
                else:
                    location_x = radius * math.cos(theta)
                    location_y = radius * math.sin(theta)
            else:
                location_x = random.random() * self.width
                location_y = random.random() * self.height
            self.position[node]=(location_x, location_y)
            self.node[node]['location_x'] = location_x
            self.node[node]['location_y'] = location_y

        self.node[0]['type'] = 0

        while relays > 0:
            node = random.choice(self.nodes())
            if self.node[node]['type'] not in [0,1]:
                self.node[node]['type'] = 1
                self.relays.add(node)
                relays = relays - 1

        for node in self:
            if self.node[node]['type'] == 2:
                self.subscribers.add(node)

        # Relays have a skeleton
        for relay in self.relays:
            self.add_edge(0, relay)

        # Populate Edge data
        for edge in self.edges():
            self.init_edge(edge[0], edge[1])

    def initialize_edges(self):
        self.primary_interference()

        # Remove dead edges
        for e in self.edges():
            if self.bottleneck_capacity(e) == 0.0:
                self.remove_edge(*e)

        # Remove dead nodes
        for n in self.nodes():
            if len(self.neighbors(n)) == 0:
                self.remove_node(n)

    def init_edge(self, src, dst):
        e = self[src][dst]
        e['distance'] = self.distance(src, dst)
        e['channels'] = {
            700 : dict(),
            2.4 : dict(),
            5.8 : dict()
        }
        for j in e['channels'].keys():
            for i in range(self.channels):
                e['channels'][j][i] = self.throughput(src, dst, freq=j);

    def prune_dead_edges(self):
        for e in self.edges():
            if self.bottleneck_capacity(e) == 0.0:
                self.remove_edge(*e)

    def save(self, filename="graph.graphml"):
        "Write out the graph to a graphml file."
        networkx.write_graphml(self, filename, prettyprint=True)

    def draw(self):
        "Draw this graph with customizations for our toolkit."
        self.update_throughput_for_nodes()
        node_size = [self.node[node]['throughput'] for node in self]
        node_size = [(n / max(node_size)) * 400 for n in node_size]
        node_color = [float(self.node[node]['type']) for node in self]
        edge_labels = dict()
        for edge in self.edges():
            edge_labels[edge] = "%0.2f km : %0.1f mbps" % (self[edge[0]][edge[1]]['distance'], 
                self.bottleneck_capacity(edge) / math.pow(10,6))

        networkx.draw(self, self.position, 
                      node_size=node_size,
                      node_color=node_color)
        networkx.draw_networkx_edge_labels(self, self.position, 
                      edge_labels=edge_labels)

    def distance(self, from_node, to_node):
        x1 = self.node[from_node]['location_x']
        y1 = self.node[from_node]['location_y']
        x2 = self.node[to_node]['location_x']
        y2 = self.node[to_node]['location_y']
        distance = math.sqrt(math.pow((x1-x2),2) + math.pow((y1-y2),2))
        return distance

    def bearing(self, from_node, to_node):
        x1 = self.node[from_node]['location_x']
        y1 = self.node[from_node]['location_y']
        x2 = self.node[to_node]['location_x']
        y2 = self.node[to_node]['location_y']
        distance = self.distance(from_node, to_node) * 1000
        angle = math.degrees(math.acos((y2-y1)/distance))
        if x2 > x1:
            return angle
        else:
            return 360.0 - angle

    def pathloss(self, from_node, to_node, theta=360.0, freq=2.4):
        distance = self.distance(from_node, to_node) * 1000
        pathloss = math.pow(10, (2 + 10 * math.log10(360.0/theta))/10.0)
        pathloss *= self.node[to_node]['receiver_gain']
        pathloss *= math.pow(wavelength[freq], 2)
        pathloss /= math.pow(4 * math.pi * distance, 2)
        pathloss = math.fabs(10 * math.log10(pathloss))

        return pathloss

    def throughput(self, from_node, to_node, theta=360.0, freq=2.4):
        pathloss = self.pathloss(from_node, to_node, theta=theta, freq=freq)

        if pathloss > 126.0:
            throughput = 0.0
        elif pathloss > 121.5:
            throughput = 10 * math.pow(10,6)
        elif pathloss > 118.75:
            throughput = 20 * math.pow(10,6)
        elif pathloss > 114.5:
            throughput = 30 * math.pow(10,6)
        elif pathloss > 113:
            throughput = 40 * math.pow(10,6)
        else:
            throughput = 45 * math.pow(10,6)

        return throughput

    def beam_covers(self, from_node, to_node, theta):
        bearing = self.bearing(from_node, to_node)
        lower_bound = bearing - (thata/2)
        upper_bound = bearing + (thata/2)
        best_bearing = from_node['best_bearing']
        if best_bearing >= lower_bound and best_bearing <= upper_bound:
            return True
        else:
            return False

    def bottleneck_capacity(self, edge):
        total_capacity = 0.0
        e = self[edge[0]][edge[1]]
        for frequency in e['channels'].keys():
            for channel, throughput in e['channels'][frequency].items():
                total_capacity += throughput
        return total_capacity

    def bottleneck_weight(self, edge):
        max_possible = 45 * math.pow(10,6) 
        max_possible *= 3  # frequencies
        max_possible *= self.channels # channels
        return(max_possible - (1.0 * self.bottleneck_capacity(edge)))
        
    def primary_interference(self):
        for i in range(len(self.nodes()) * self.channels):
            src = random.choice(self.nodes())
            rand_freq = random.choice(signal_range.keys())
            rand_channel = random.randint(0, self.channels-1)
            ns_range = signal_range[rand_freq]
            for dst in self.nodes():
                if self.distance(src, dst) < ns_range:
                    for e in networkx.edges(self, [dst]):
                        self[e[0]][e[1]]['channels'][rand_freq][rand_channel] = 0.0

    def update_throughput_for_nodes(self):
        total_throughput = 0.0
        for n in self.nodes():
            neighbors = self.neighbors(n)
            for neighbor in neighbors:
                node_to_neighbor = self.throughput(n, neighbor, theta=self.theta)
                neighbor_to_node = self.throughput(neighbor, n, theta=self.theta)
                node = self.node[n]                    
                neighbor = self.node[neighbor]
                node['in_throughput'] += neighbor_to_node
                node['out_throughput'] += node_to_neighbor
                total_throughput += node_to_neighbor + neighbor_to_node
                node['throughput'] = node['in_throughput'] + node['out_throughput']
        return total_throughput

    def interference(self):
        for edge1 in self.edges():
            for edge2 in self.edges():
                if edge1 != edge2:
                    v1e1 = edge1[0]
                    v2e1 = edge1[1]
                    v1e2 = edge2[0]
                    v2e2 = edge2[1]
                    distances = [ self.distance(v1e1, v1e2),
                                  self.distance(v1e1, v2e2),
                                  self.distance(v1e2, v1e2),
                                  self.distance(v1e2, v2e2) ]
                    distances.sort()
                    shortest = distances[0]
                    
                    vertices = set()
                    vertices.update([v1e1, v2e1, v1e2, v2e2])

                    for i in [700, 2.4, 5.8]:
                        for j in range(self.channels):
                            if self[edge1[0]][edge1[1]]['channels'][i][j] > 0.0 and self[edge2[0]][edge2[1]]['channels'][i][j] > 0.0:
                                if shortest < signal_range[i] or len(vertices) < 4:
                                    self.interference[edge1][edge2][i][j] = True
                                    self.interference[edge2][edge1][i][j] = True
                                else:
                                    self.interference[edge1][edge2][i][j] = False
                                    self.interference[edge2][edge1][i][j] = False

    def beamsets(self):
        relay_bearings = OrderedDict()
        for relay in self.relays:
            relay_bearings[relay] = OrderedDict()
            for subscriber in self.subscribers:
                bearing = self.bearing(relay, subscriber)
                throughput = self.throughput(relay, subscriber,
                                                       self.theta)
                if throughput > 0.0:
                     relay_bearings[relay][bearing] = subscriber
            
            sorted_subscribers = relay_bearings[relay]
            
            for bearing in range(360):
                right = (bearing + self.theta / 2) % 360
                left = (bearing - self.theta / 2) % 360
                beamset = set()
                for b,s in sorted_subscribers.items():
                    if b > left and b < right:
                        beamset.add(s)
                if len(beamset) > 0:
                    self.beamset[relay][bearing] = beamset

    def beam_index(self, src, dst, beams=8):
        bearing = self.bearing(src, dst)
        beam = int(1.0 + math.floor(math.radians(bearing) / ((2.0 * math.pi) / beams)))
        return beam
