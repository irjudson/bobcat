#
#  Beam Scheduling Algorithms
#  (c) 2013 Ivan R. Judson / Montana State University
#

def greedy1(network):
    for sub in network.subscribers:
        network.node[sub]['preferred_relay'] = None

    for r in network.relays:
        vv = dict()
        relay = network.node[r]
        relay['allocated_channels'] = 0
        for sub in network.subscribers:
            subscriber = network.node[sub]
            spr = network.node[sub]['preferred_relay']
            ql = network.node[sub]['queue_length']
            rtput = ql * min(ql, 
                             network.throughput(r, sub, 
                                theta=network.theta) * network.slot_length)
            if spr is None:
                vv[sub] = rtput
            else:
                vv[sub] = rtput - ql * min(ql, network.throughput(spr, sub, theta=network.theta) * network.slot_length)

        allocated_channels = 0
        for s in sorted(vv, key=vv.get, reverse=True):
            if vv[s] > 0.0 and allocated_channels < network.channels:
                network.node[s]['preferred_relay'] = r
                allocated_channels += 1

    # Find beam sets
    for r in network.relays:
        relay = network.node[r]
        relay['allocated_channels'] = 0
        best_beamset_value = 0.0
        for bearing,subscribers in network.beamset[r].items():
            beamset_value = 0.0
            for sub in subscribers:
                subscriber = network.node[sub]
                if subscriber['preferred_relay'] == r:
                    ql = subscriber['queue_length']
                    beamset_value += ql * min(ql, network.throughput(r, sub, network.theta) * network.slot_length)
            if beamset_value >= best_beamset_value:
                relay['best_bearing'] = (bearing, beamset_value)
                best_beamset_value = beamset_value

    for s in network.subscribers:
        if network.node[s]['preferred_relay'] is not None:
            r = network.node[s]['preferred_relay']
            relay = network.node[r]
            best_beamset = network.beamset[r][relay['best_bearing'][0]]
            if s in best_beamset:
                relay['allocated_channels'] += 1
            else:
                network.node[s]['preferred_relay'] = None

    for s in network.subscribers:
        subscriber = network.node[s]
        if subscriber['preferred_relay'] is None:
            vv = dict()
            for r in network.relays:
                relay = network.node[r]
                beamset = network.beamset[r][relay['best_bearing'][0]]
                if s in beamset:
                    ql = subscriber['queue_length']
                    vv[r] = ql * min(ql, network.throughput(r, s, network.theta) * network.slot_length)
                else:
                    vv[r] = 0.0
                                          
            for r in sorted(vv, key=vv.get, reverse=True):
                value = vv[r]
                relay = network.node[r]
                if value > 0.0 and relay['allocated_channels'] < network.channels:
                    subscriber['preferred_relay'] = r
                    relay['allocated_channels'] += 1
                    break


    objective = 0.0
    for s in network.subscribers:
        subscriber = network.node[s]
        if subscriber['preferred_relay'] is not None:
            pri = subscriber['preferred_relay']
            beamset = network.beamset[pri][network.node[pri]['best_bearing'][0]]
            if s in beamset:
                ql = subscriber['queue_length']
                objective += ql * min(ql, network.throughput(pri, s, network.theta) * network.slot_length)
                network.add_edge(pri, s)
                network.init_edge(pri, s)

    return objective

def greedy2(network):
    for sub in network.subscribers:
        network.node[sub]['preferred_relay'] = None

    for r in network.relays:
        vv = dict()
        relay = network.node[r]
        relay['allocated_channels'] = 0
        for sub in network.subscribers:
            subscriber = network.node[sub]
            spr = network.node[sub]['preferred_relay']
            ql = network.node[sub]['queue_length']
            rtput = ql * min(ql, 
                             network.throughput(r, sub, 
                                theta=network.theta) * network.slot_length)
            if spr is None:
                vv[sub] = rtput
            else:
                vv[sub] = rtput - ql * min(ql, network.throughput(spr, sub, theta=network.theta) * network.slot_length)

        allocated_channels = 0
        for s in sorted(vv, key=vv.get, reverse=True):
            if vv[s] > 0.0 and allocated_channels < network.channels:
                network.node[s]['preferred_relay'] = r
                allocated_channels += 1

        # Find beam sets
        relay['allocated_channels'] = 0
        best_beamset_value = 0.0
        for bearing,subscribers in network.beamset[r].items():
            beamset_value = 0.0
            for sub in subscribers:
                subscriber = network.node[sub]
                if subscriber['preferred_relay'] == r:
                    ql = subscriber['queue_length']
                    beamset_value += ql * min(ql, network.throughput(r, sub, network.theta) * network.slot_length)
            if beamset_value >= best_beamset_value:
                relay['best_bearing'] = (bearing, beamset_value)
                best_beamset_value = beamset_value

    for s in network.subscribers:
        subscriber = network.node[s]
        if subscriber['preferred_relay'] is None:
            vv = dict()
            for r in network.relays:
                relay = network.node[r]
                beamset = network.beamset[r][relay['best_bearing'][0]]
                if s in beamset:
                    ql = subscriber['queue_length']
                    vv[r] = ql * min(ql, network.throughput(r, s, network.theta) * network.slot_length)
                else:
                    vv[r] = 0.0
                                          
            for r in sorted(vv, key=vv.get, reverse=True):
                value = vv[r]
                relay = network.node[r]
                if value > 0.0 and relay['allocated_channels'] < network.channels:
                    subscriber['preferred_relay'] = r
                    relay['allocated_channels'] += 1
                    break


    objective = 0.0
    for s in network.subscribers:
        subscriber = network.node[s]
        if subscriber['preferred_relay'] is not None:
            pri = subscriber['preferred_relay']
            beamset = network.beamset[pri][network.node[pri]['best_bearing'][0]]
            if s in beamset:
                ql = subscriber['queue_length']
                objective += ql * min(ql, network.throughput(pri, s, network.theta) * network.slot_length)
                network.add_edge(pri, s)
                network.init_edge(pri, s)

    return objective

import cplex
from cplex.exceptions import CplexSolverError

def optimal(network):
    model = cplex.Cplex()

    try:
        model.write("bsrap.lp")
        model.solve()
    except CplexSolverError, e:
        print "Exception raised during solve: " + e
    else:
        print "Solved optimal solution."