#
#  Beam Scheduling Algorithms
#  (c) 2013 Ivan R. Judson / Montana State University
#

import pprint

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
    r = dict()
    THRESHOLD = 0.0001
    model = cplex.Cplex()
    model.set_problem_name("BS-RAP")

    # Initialize y
    names = ["y(%d)" % i for i in network.subscribers]
    lb = [0.0] * len(names)
    ub = [network.node[i]['queue_length'] for i in network.subscribers]
    ty = [model.variables.type.continuous] * len(names)
    model.variables.add(names=names, lb=lb, ub=ub, types=ty)

    # Set the objective
    coefficients = [(names.index("y(%d)" % i), network.node[i]['queue_length']) for i in network.subscribers]
    model.objective.set_linear(coefficients)
    model.objective.set_sense(model.objective.sense.maximize)

    # 1. Initialize x
    for i in network.relays:
        r[i] = dict()
        for j in network.subscribers:
            r[i][j] = network.throughput(i, j, network.theta) * network.slot_length
            model.variables.add(names=["x(%d)(%d)" % (i, j)], lb=[0], ub=[1], types=[model.variables.type.integer])
            if r[i][j] < THRESHOLD:
                model.linear_constraints.add(lin_expr=[["x(%d)(%d)" % (i,j)],[1.0]], rhs=[0], senses=["E"])

    # 2. Initialize s
    for i in network.relays:
        for l in range(len(network.beamset[i])):
            model.variables.add(names=["s(%d)(%d)" % (i, l)], lb=[0], ub=[1], types=[model.variables.type.integer])

    # 3. Constraint 5
    rows = list()
    rhs = list()
    senses = ""
    for i in network.subscribers:
        rows.append([["y(%d)" % i ], [1.0]])
        rhs.append(network.node[i]['queue_length'])
        senses += "L"
    model.linear_constraints.add(lin_expr=rows, rhs=rhs, senses=senses)

    # 4. Constraint 6
    rows = list()
    rhs = list()
    senses = ""
    for j in network.subscribers:
        row = ["y(%d)" % j]
        coefficients = [-1.0]
        for i in network.relays:        
            row.append("x(%d)(%d)" % (i,j))
            coefficients.append(r[i][j])
        rows.append([row, coefficients])
        rhs.append(0.0)
        senses += "L"
    model.linear_constraints.add(lin_expr=rows, rhs=rhs, senses=senses)

    # 5. Constraint 7
    names = list()
    for j in network.subscribers:
        for i in network.relays:
            names.append("x(%d)(%d)" % (i,j))
    model.linear_constraints.add(lin_expr=[[names, [1.0] * len(names)]], rhs=[1.0], senses=["L"])

    # 6. Constraint 8
    names = list()
    for i in network.relays:
        for l in range(len(network.beamset[i])):
            names.append("s(%d)(%d)" % (i, l))
    model.linear_constraints.add(lin_expr=[[names, [1.0] * len(names)]], rhs=[1.0], senses=["E"])


    # 7.  Constaint 9
    rhs = list()
    rows = list()
    senses = ""
    for i in network.relays:
        for j in network.subscribers:
            lhs = ["x(%d)(%d)" % (i,j)]
            coefficients = [-1.0]
            for l in range(len(network.beamset[i])):
                if l in network.beamset[i] and j in network.beamset[i][l]:
                    lhs.append("s(%d)(%d)" % (i, l))
                    coefficients.append(1.0)
            rows.append([lhs, coefficients])
            rhs.append(0.0)
            senses += "G"
    model.linear_constraints.add(lin_expr=rows, rhs=rhs, senses=senses)

    # 8. Constraint 10
    names = list()
    for i in network.relays:
        for j in network.subscribers:
            names.append("x(%d)(%d)" % (i,j))
    model.linear_constraints.add(lin_expr=[[names, [1.0] * len(names)]], rhs=[network.channels], senses=["L"])

    # Solve the problem, return the value    
    try:
#        model.write("bsrap.lp")
        model.write("bsrap.mps")
        model.set_results_stream(None)
        model.solve()
        return(model.solution.get_objective_value())
    except CplexSolverError as e:
        print("Exception raised during solve: ", e)
    else:
        print("Solved optimal solution.")
