#
# Joint Routing and Channel Selection
# (c) 2013 Ivan R. Judson / Montana State University
#

import sys
import pprint

class PCS:
    def __init__(self, other=None):

        if other is not None:
            self.path = other.path[:]
            self.throughput = other.throughput
            self.path_channel_set = Path(selected=other.path_channel_set.selected[:],
                throughput=other.path_channel_set.throughput)
        else:
            self.path = list()
            self.throughput = sys.float_info.max
            self.path_channel_set = Path()

    def __repr__(self):
        return "-- %s -- %s" % (self.path, self.path_channel_set)

class Path:
    def __init__(self, selected=list(), throughput=sys.float_info.max):
        # list of EdgeChannel Object Sets
        self.selected = selected

        # Throughput of this Path
        self.throughput = throughput

    def __repr__(self):
        return "/ %e / [ %s ]" % (self.throughput, self.selected)

class Link:
    def __init__(self, edge=None, freq=-1.0, channel=-1):
        self.edge = edge
        self.freq = freq
        self.channel = channel

    def __repr__(self):
        return "/ (%d, %d), %f, %d /" % (self.edge[0], self.edge[1], self.freq, self.channel)

    def __eq__(self, other):
        if self.edge == other.edge and self.freq == other.freq and self.channel == other.channel:
            return True
        return False

    def __hash__(self):
        return hash((self.edge, self.freq, self.channel))

# Joint Routing and Channel Selection
def rcs_add_edge_weights(network, src, dst):
    dmax = sys.float_info.min
    dmin = sys.float_info.max

    for edge in network.edges():
        avge_dist = (network.distance(edge[0], src) + network.distance(edge[1], dst)) / 2.0
        if avge_dist < dmin:
            dmin = avge_dist
        if avge_dist > dmax:
            dmax = avge_dist

    for edge in network.edges():
        avge_dist = (network.distance(edge[0], src) + network.distance(edge[1], dst)) / 2.0
        network[edge[0]][edge[1]]['weight'] = (1.0 + (dmax - d) / (dmax-dmin)) / 2.0
        network[edge[0]][edge[1]]['bottleneck_weight'] = network.bottleneck_weight(edge)

def find_available_links(network, path):
    available_links = dict()
    for i in range(len(path)):
        available_links[i] = set()
        edge = path[i]
        for freq in network.FREQUENCIES:
            for c in range(network.channels):
                if network[edge[0]][edge[1]]['channels'][freq][c] > 0.0:
                    available_links[i].add(Link(edge, freq, c))
    return available_links

def check_interference(network, i, j, f, c):
    if f in network.interference[i][j]:
        if c in network.interference[i][j][f]:
            return network.interference[i][j][f][c]

    return False

def compute_bridging_set(network, available_links, idx):
    bridging_set = set()
    bridging_set.update(available_links[idx])
    left = set()
    right = set()
    for i in available_links.keys():
        if i <= idx:
            left.update(available_links[i])
        else:
            right.update(available_links[i])

    for l in left:
        for r in right:
            if l == r and check_interference(network, l, r, l.freq, l.channel):
                bridging_set.update([l, r])

    return bridging_set

def encode_set(network, set_in, subset):
    idx = 0
    k = 0
    for el in set_in:
        if el in subset:
            idx += (1 << k)
        k += 1
    return idx

def decode_set(network, set_in, idx):
    subset = set()
    k = 0
    for el in set_in:
        value = idx >> k
        if (idx >> k ) % 2 == 1:
            subset.add(el)
        k += 1
    return subset

def max_clique(network, links, loc, freq, channel):
    max_size = 0
    i = 0
    while i < len(links):
        j = i + 1
        while j < len(links) and check_interference(network, links[i], links[j], freq, channel):
            j += 1
        size = j - i + 1
        if i <= loc and j <= loc and size > max_size:
            max_size = size
        i += 1
    return max_size

def throughput_for_test_path(network, old_path, test_path_channel_set):
    path_len = len(old_path)
    path = old_path[:]

    if test_path_channel_set is None:
        return 0.0

    for i in range(0, path_len):
        max_clique_size = 1
        if path_len > 1:
            max_clique_size = 2
        if i > 1 and i < path_len:
            left = set()
            right = set()
            for j in range(0, len(test_path_channel_set.selected)):
                if j <= i:
                    left.update(test_path_channel_set.selected[j])
                else:
                    right.update(test_path_channel_set.selected[j])                    

            if len(left & right) > 0:
                max_clique_size = 3

        link_throughput = 0.0

        for link in test_path_channel_set.selected[i]:
            links = list()
            for j in range(0, path_len):
                test_link = Link(path[j], link.freq, link.channel)
                if test_link in test_path_channel_set.selected[j]:
                    links.append(j)

            max_channel_clique_size = max(max_clique_size, max_clique(network, links, i, link.freq, link.channel))
            link_throughput += (network[link.edge[0]][link.edge[1]]['channels'][link.freq][link.channel] / max_channel_clique_size)
    
        test_path_channel_set.throughput = min(test_path_channel_set.throughput, link_throughput)

    return test_path_channel_set.throughput

def throughput_for_path(network, path, bridging_set, path_len, available_links):
    i = len(path.selected)-1
    max_clique_size = 1
    if path_len > 1: 
        max_clique_size = 2

    if i > 1 and i < path_len - 1:
        left = set()
        for link in path.selected[i-1]:
            left.add((link.freq, link.channel))
        right = set()
        for link in available_links[i+1] & bridging_set:
            right.add((link.freq, link.channel))
        if len(left & right) > 0:
            max_clique_size = 3

    link_throughput = 0.0

    for link in path.selected[i]:
        links = list()
        for j in range(0, i):
            edge = network.edges()[j]
            test_link = Link(edge, link.freq, link.channel)
            if test_link in path.selected[j]:
                links.append(j)

        links.append(i)

        for link2 in bridging_set:
            if link.channel == link2.channel and link.freq == link2.freq \
                    and link2.edge[0] > i:
                links.append(link.edge)

        max_channel_clique_size = max(max_clique_size, max_clique(network, links, i, link.freq, link.channel))
        link_throughput += (network[link.edge[0]][link.edge[1]]['channels'][link.freq][link.channel] / max_channel_clique_size)
    
    path.throughput = min(path.throughput, link_throughput)

def select_channels(network, path):
    if path is None or len(path) == 0:
        return 0.0

    path_len = len(path)
    bridging_set = dict()
    bridging_set_count = dict()

    # Find all available freq/channels for each edge along the path
    available_links = find_available_links(network, path)

    # Compute bridging sets for each edge on the path
    for i in range(0, len(path)):
        bridging_set[i] = compute_bridging_set(network, available_links, i)
        bridging_set_count[i] = pow(2, len(bridging_set[i]))

    # Initialize Dynamic programming
    best_path_channel_set = dict()
    for i in range(0, len(path)):
        best_path_channel_set[i] = dict()
    for k in range(bridging_set_count[0]):
        best_path_channel_set[0][k] = Path()

    for i in range(1, len(path)):
        new_part = bridging_set[i].copy() - bridging_set[i-1]
        old_part = bridging_set[i].copy() & bridging_set[i-1]

        for k in range(bridging_set_count[i-1]):
            previous_best = best_path_channel_set[i-1][k]
            previous_bridging_set = decode_set(network, bridging_set[i-1], k)
            copy_of_old_part = old_part.copy() & previous_bridging_set
            if len(new_part) == 0:
                next_bridging_set_idx = encode_set(network, bridging_set[i], copy_of_old_part)
                test_path = Path(selected=previous_best.selected[:])
                test_path.selected.append(previous_bridging_set.selected.copy() & available_links[i-1])
                throughput_for_path(network, test_path, previous_bridging_set, path_len, available_links)
                if next_bridging_set_idx not in best_path_channel_set[i] or \
                   test_path.throughput > best_path_channel_set[i][next_bridging_set_idx].throughput:
                    best_path_channel_set[i][next_bridging_set_idx] = test_path
            else:
                for l in range(1 << len(new_part)):
                    next_bridging_subset = decode_set(network, new_part, l) 
                    next_bridging_subset.update(old_part.copy())
                    next_bridging_set_idx = encode_set(network, bridging_set[i], next_bridging_subset)
                    test_path = Path(selected=previous_best.selected[:])
                    test_path.selected.append(previous_bridging_set.copy() & available_links[i-1])
                    throughput_for_path(network, test_path, next_bridging_subset, path_len, available_links)
                    if next_bridging_set_idx not in best_path_channel_set[i] or \
                       test_path.throughput > best_path_channel_set[i][next_bridging_set_idx].throughput:
                        best_path_channel_set[i][next_bridging_set_idx] = test_path


    optimal_path = None
    empty = set()
    for k in range(bridging_set_count[path_len - 1]):
        next_bridging_subset = decode_set(network, bridging_set[path_len - 1], k)
        previous_best = best_path_channel_set[path_len - 1][k]
        test_path = Path(selected=previous_best.selected[:], 
                         throughput=previous_best.throughput)
        test_path.selected.append(next_bridging_subset.copy() & available_links[path_len - 1])
        throughput_for_path(network, test_path, empty, path_len, available_links)
        if optimal_path is None or test_path.throughput > optimal_path.throughput:
            optimal_path = test_path

    if optimal_path:
        return optimal_path.throughput

    return 0.0

def select_channels_greedy(network, path):
    path_len = len(path)

    available_links = find_available_links(network, path)
    greedy_path = Path(selected=[available_links[0]])

    for i in range(1, path_len):
        remove_set = set()
        for link in greedy_path.selected[i-1]:
            new_link = Link(i, link.freq, link.channel)
            remove_set.add(new_link)
        next_set = available_links[i] - remove_set
        if len(next_set) > 0:
            greedy_path.selected.append(next_set)
        else:
            print(type(available_links[i]))
            greedy_path.selected.append(available_links[i])

    for i in range(0, path_len):
        max_clique_size = 1
        if path_len > 1:
            max_clique_size = 2

        if i > 1 and i < path_len - 1:
            left = set()
            for link in greedy_path.selected[i-1]:
                left.add((link.freq, link.channel))
            right = set()
            for link in greedy_path.selected[i+1]:
                right.add((link.freq, link.channel))
            left &= right
            if len(left) > 0:
                max_clique_size = 3

        link_throughput = 0.0
        link_nums = list()
        for link in greedy_path.selected[i]:
            link_nums = list()
            link_nums.append(i)

            for j in range(0, i):
                test_link = Link(j, link.freq, link.channel)
                if test_link in greedy_path.selected[j]:
                    link_nums.append(j)
            for j in range(i+1,path_len):
                test_link = Link(j, link.freq, link.channel)
                if test_link in greedy_path.selected[j]:
                    link_nums.append(j)
            max_clique_size = max(max_clique_size, 
                                  max_clique(network, link_nums, i, link.freq, link.channel))
            link_throughput += network[link.edge[0]][link.edge[1]]['channels'][link.freq][link.channel]
        greedy_path.throughput = min(greedy_path.throughput, link_throughput)
    return greedy_path.throughput

def combinations(current, combos):
    combos.update(current)
    for o in current:
        next = current[:]
        next.remove(o)
        if len(next) > 0:
            combinations(next[:], combos)

def vertices_for_path(path):
    "path is a list of tuples of edges"
    vertices = set()
    [vertices.update(list(edge)) for edge in path]
    return vertices

def rcs_path(network, src, dst, consider=10):
    # Initialize
    for node in network.nodes():
        network.node[node]['rcs_paths'] = dict()
        if node == src:
            network.node[node]['rcs_paths'][0.0] = PCS()

    for i in range(len(network.nodes())):
        for e in network.edges():
            u = e[0]
            v = e[1]
            chset = set()
            cset = list()

            for freq in network.FREQUENCIES:
                for channel in range(network.channels):
                    if network[u][v]['channels'][freq][channel] > 0.0:
                        cset.append((freq, channel))

            combinations(cset, chset)

            for thpt, opcs in network.node[u]['rcs_paths'].items():
                new_path = opcs.path[:]
                if len(opcs.path) == i and v not in vertices_for_path(opcs.path):
                    for chs in chset:
                        npcs = PCS(other=opcs)
                        npcs.path.append(e)
                        nls = set()
                        for freq in network.FREQUENCIES:
                            for channel in range(0, network.channels):
                                nls.add(Link(e, freq, channel))
                        npcs.path_channel_set.selected.append(nls)
                        thpt = throughput_for_test_path(network, npcs.path, npcs.path_channel_set)
                        network.node[v]['rcs_paths'][thpt] = npcs
                        if len(network.node[v]['rcs_paths'].keys()) > consider:
                            del network.node[v]['rcs_paths'][min(network.node[v]['rcs_paths'].keys())]

            for thpt, opcs in list(network.node[v]['rcs_paths'].items()):
                new_path = opcs.path[:]
                if len(opcs.path) == i and u not in vertices_for_path(opcs.path):
                    for chs in chset:
                        npcs = PCS(other=opcs)
                        npcs.path.append(e)
                        nls = set()
                        for freq in network.FREQUENCIES:
                            for channel in range(0, network.channels):
                                nls.add(Link(e, freq, channel))
                        npcs.path_channel_set.selected.append(nls)
                        thpt = throughput_for_test_path(network, npcs.path, npcs.path_channel_set)
                        network.node[v]['rcs_paths'][thpt] = npcs
                        if len(network.node[u]['rcs_paths'].keys()) > consider:
                            del network.node[u]['rcs_paths'][min(network.node[u]['rcs_paths'].keys())]


    if len(network.node[dst]['rcs_paths']) == 0:
        return None

    return network.node[dst]['rcs_paths'][max(network.node[dst]['rcs_paths'].keys())]
