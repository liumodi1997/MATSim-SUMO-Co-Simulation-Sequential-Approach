import csv
import gzip
import xml.etree.ElementTree as ET
import os
import ClassDefine as cd


def transfer_link(file, sumo_link):
    print('Transforming MATSim output (read by line)\n')
    # veh[i] = [  [link_1,time_1(depart_time)], [link_2,time_2], ..., [link_k,time_k], [-1, arr_time] ]
    veh = []
    num_event = 0
    num_veh = 0
    n = 0

    ending = file[-3:]
    if ending == ".gz":
        print('Loading event gz file')
        gfile = gzip.open(file, 'r')
    else:
        print('Loading event xml file')
        gfile = open(file, 'r')

    line = gfile.readline()
    if ending == ".gz":
        line = str(line, encoding="utf8")
    start_time = 10000000
    end_time = -1

    # build the list, whose index is the Vehicle_ID, and whose context is routes and time
    # record the start time and end time of all events
    # build the list of class event, which includes the events in SUMO-area
    sumo_events = []

    while line != '':
        segments = line.split('" ')
        first = segments[0]
        if first[1:7] != "<event":
            line = gfile.readline()
            if ending == ".gz":
                line = str(line, encoding="utf8")
            continue
        child = cd.event_ele(segments)
        line = gfile.readline()
        if ending == ".gz":
            line = str(line, encoding="utf8")
        num_event += 1
        veh_id = child.id
        type = child.type
        n += 1
        if n % 10000000 == 0:
            print(n)

        if type == "vehicle enters traffic" or type == "entered link" or type == "vehicle leaves traffic" or type == "left link":
            link_id = child.link
            if link_id in sumo_link:
                one_event = cd.event(veh_id, type, child.time, link_id, child.pos)
                sumo_events.append(one_event)
                if child.time < start_time:
                    start_time = child.time
                if child.time > end_time:
                    end_time = child.time
    start_time = 0
    start_time = 0
    end_time = 86400
    return sumo_events, start_time, end_time

class measure_ele:
    def __init__(self, link_id, length):
        self.link_id = link_id
        self.length = length
        self.enter_num = []
        self.leave_num = []
        self.num = []
        self.queue = []
        self.avg_speed = []


def measure_link(sumo_events, time_interval, matsim_link_sumo):
    count = 0
    i_interval = 1
    start_time_intervals = [0]
    # link_measure = { link_id: measure_ele }
    link_measure = {}
    start_time = 0
    for one_event in sumo_events:
        count += 1
        time1 = one_event.time
        link_id = one_event.link

        if link_id not in link_measure:
            link_length = matsim_link_sumo[link_id].length
            ele = measure_ele(link_id, link_length)
            link_measure[link_id] = ele

        # if the time is larger than the end time of ith-interval, then the next interval will be considered
        while time1 > (start_time + i_interval * time_interval):
            start_time_intervals.append(start_time + i_interval * time_interval)
            i_interval += 1

        # update each measurement's list member
        while len(link_measure[link_id].enter_num) < i_interval:
            link_measure[link_id].enter_num.append(0)
            link_measure[link_id].leave_num.append(0)
            link_measure[link_id].num.append(0)
            link_measure[link_id].avg_speed.append(0)

        type = one_event.type
        veh_id = one_event.veh_id
        pos = one_event.pos


        if type == 'entered link' or type == "vehicle enters traffic":
            link_id = one_event.link
            if one_event.pos != 1:
                link_measure[link_id].enter_num[i_interval - 1] += 1
                link_measure[link_id].queue.append([one_event.veh_id, time1, one_event.pos, one_event.type, link_id])

        if type == 'left link' or type == 'vehicle leaves traffic':
            link_id = one_event.link
            veh_id = one_event.veh_id
            enter_ele = []
            enter_ind = -1
            for queue_ele in link_measure[link_id].queue:
                enter_ind += 1
                if queue_ele[0] == veh_id:
                    enter_ele = queue_ele
                    break
            if enter_ele != []:
                if time1 - enter_ele[1] != 0:
                    link_measure[link_id].avg_speed[i_interval - 1] += link_measure[link_id].length / (time1 - enter_ele[1])
                    link_measure[link_id].leave_num[i_interval - 1] += 1
                else:
                    link_measure[link_id].leave_num[i_interval - 1] += 1
                q = link_measure[link_id].queue
                del q[enter_ind]
    return link_measure


def create_csv(link_file, edge_file, comp_file, link_measure, edge_measure, comp_measure, time_interval, start_time, end_time):
    # Link
    print(link_file)
    f = open(link_file, 'w+', encoding='UTF8', newline='')
    writer = csv.writer(f)
    # 'time_interval', 'link_id', 'length', 'enter_num', 'leave_num', 'avg_speed'
    header = ['time_interval']
    time1 = start_time
    i_interval = 0
    for link_id in link_measure:
        header.append(link_id)
        header.append('length')
        header.append('enter_num')
        header.append('leave_num')
        header.append('avg_speed')

    writer.writerow(header)

    while time1 < end_time:
        # line = [str(time1) + '-' + str(time1 + time_interval)]
        line = [str(time1)]
        for link_id in link_measure:
            ele = link_measure[link_id]
            if i_interval < len(ele.enter_num):
                line.append(ele.link_id)
                line.append(str(ele.length))
                line.append(str(ele.enter_num[i_interval]))
                line.append(str(ele.leave_num[i_interval]))
                if ele.leave_num[i_interval] != 0:
                    line.append(str(ele.avg_speed[i_interval] / ele.leave_num[i_interval]))
                else:
                    line.append(0)
            else:
                line.append(ele.link_id)
                line.append(str(ele.length))
                line.append(0)
                line.append(0)
                line.append(0)
        i_interval += 1
        time1 += time_interval
        writer.writerow(line)

    f.close()

    # Edge
    print(edge_file)
    f = open(edge_file, 'w+', encoding='UTF8', newline='')
    writer = csv.writer(f)
    # 'time_interval', 'edge_id', 'length', 'enter_num', 'leave_num', 'avg_speed', 'timeTravel', 'timeLoss'
    header = ['time_interval']
    time1 = start_time
    i_interval = 0
    for edge_id in edge_measure:
        header.append(edge_id)
        header.append('length')
        header.append('enter_num')
        header.append('leave_num')
        header.append('avg_speed')
        header.append('timeTravel')
        header.append('timeLoss')
    writer.writerow(header)

    time1 = 0
    while time1 < end_time:
        line = [str(time1)]
        for edge_id in edge_measure:
            ele = edge_measure[edge_id]
            if i_interval < len(ele.enter_num):
                line.append(ele.edge_id)
                line.append(str(ele.length))
                line.append(str(ele.enter_num[i_interval]))
                line.append(str(ele.leave_num[i_interval]))
                line.append(str(ele.avg_speed[i_interval]))
                line.append(str(ele.travelTime[i_interval]))
                line.append(str(ele.timeLoss[i_interval]))
            else:
                line.append(ele.link_id)
                line.append(str(ele.length))
                line.append(0)
                line.append(0)
                line.append(0)
                line.append(0)
                line.append(0)
        i_interval += 1
        time1 += time_interval
        writer.writerow(line)

    f.close()

    # Compare file
    f = open(comp_file, 'w+', encoding='UTF8', newline='')
    writer = csv.writer(f)
    # 'time_interval', 'link_id', 'edge_id', 'len_link', 'len_edge', 'comp_enter_num', 'comp_leave_num', 'comp_avg_speed', 'comp_timeTravel', 'comp_timeTravel_perMeter', 'comp_flow'
    header = ['time_interval']
    time1 = start_time
    i_interval = 0
    for link_id in comp_measure:
        header.append("link_id")
        header.append("edge_id")
        header.append('len_link')
        header.append('len_edge')
        header.append('comp_enter_num')
        header.append('comp_leave_num')
        header.append('comp_avg_speed')
        header.append('comp_travelTime')
        header.append('comp_travelTime_perMeter')
        header.append('comp_flow')
    writer.writerow(header)

    time1 = 0
    while time1 < end_time:
        line = [str(time1)]
        for link_id in comp_measure:
            ele = comp_measure[link_id]
            if i_interval < len(ele.comp_enter):
                line.append(ele.link_id)
                line.append(ele.edge_id)
                line.append(str(ele.link_len))
                line.append(str(ele.edge_len))
                line.append(str(ele.comp_enter[i_interval]))
                line.append(str(ele.comp_leave[i_interval]))
                line.append(str(ele.comp_speed[i_interval]))
                line.append(str(ele.comp_travelTime[i_interval]))
                line.append(str(ele.comp_travelTime_per_meter[i_interval]))
                line.append(str(ele.comp_flow[i_interval]))
            else:
                line.append(ele.link_id)
                line.append(ele.edge_id)
                line.append(str(ele.link_len))
                line.append(str(ele.edge_len))
                line.append(0)
                line.append(0)
                line.append(0)
                line.append(0)
                line.append(0)
                line.append(0)
        i_interval += 1
        time1 += time_interval
        writer.writerow(line)

    f.close()


# Measure SUMO simulations with netstate file
def measure_netstate(xml_file, sumo_edge, time_interval):
    tree = ET.parse(xml_file)
    root = tree.getroot()
    edge_measure = {}
    start_time_intervals = []
    i_interval = -1
    for timestep_node in root:

        if timestep_node.tag != "netstate":
            continue

        time1 = timestep_node.get('time')
        if i_interval == -1:
            start_time_intervals.append(time1)
            start_time = time1
            i_interval += 1

        # if the time is larger than the end time of ith-interval, then the next interval will be considered
        while time1 > (start_time + (i_interval + 1) * time_interval):
            start_time_intervals.append(start_time + (i_interval + 1) * time_interval)
            i_interval += 1

        for edge_node in timestep_node:
            edge_id = edge_node.get('id')
            if edge_measure[edge_id] is None:
                edge_info = sumo_edge[edge_id]
                edge_measure[edge_id] = measure_ele(edge_id, edge_info[2])
            for lane_node in edge_node:
                lane_id = lane_node.get('id')
                for vehicle_node in lane_node:
                    veh_id = vehicle_node.get('id')
                    pos = vehicle_node.get('pos')
                    speed = vehicle_node.get('speed')


def load_edge_measurement(xml_file, sumo_edge, time_interval):
    tree = ET.parse(xml_file)
    root = tree.getroot()
    # edge_measure = { edge_id: cls edgemeasure_ele }
    edge_measure = {}
    i_interval = 0
    for interval_node in root:

        begin_time = interval_node.get('begin')
        end_time = interval_node.get('end')
        for edge_node in interval_node:
            edge_id = edge_node.get('id')

            try:
                _ = edge_measure[edge_id]
            except KeyError:
                edge_info = sumo_edge[edge_id]
                edge_measure[edge_id] = cd.edgemeasure_ele(edge_id, edge_info[2])

            entered = int(edge_node.get('entered'))
            left = int(edge_node.get('left'))
            sampleSeconds = edge_node.get('sampledSeconds')

            edge_measure_ele = edge_measure[edge_id]
            edge_measure_ele.enter_num.append(entered)
            edge_measure_ele.leave_num.append(left)
            edge_measure_ele.avg_speed.append(0)
            edge_measure_ele.travelTime.append(0)
            edge_measure_ele.timeLoss.append(0)

            if float(sampleSeconds) != 0:
                travelTime = edge_node.get('traveltime')
                timeLoss = edge_node.get('timeLoss')
                speed = edge_node.get('speed')
                edge_measure_ele.avg_speed[i_interval] = speed
                edge_measure_ele.travelTime[i_interval] = travelTime
                edge_measure_ele.timeLoss[i_interval] = timeLoss

        i_interval += 1
    return edge_measure


def compare_measurement(link_measure, edge_measure, link_edge_dic, time_interval):
    comp_measure = {}
    for link in link_measure:
        edge = link_edge_dic[link]
        edge = edge[0]
        if edge == 'null':
            continue
        link_info = link_measure[link]
        try:
            edge_info = edge_measure[edge]
        except TypeError:
            print(f"link={link} edge={edge}")
        comp_ele = cd.compare_ele(link_info, edge_info, time_interval)
        comp_measure[link] = comp_ele
    return comp_measure


def update_enter_leave(veh_file, edge_measure, time_interval):
    tree = ET.parse(veh_file)
    root = tree.getroot()

    for child in root:
        if child.tag == "vehicle":
            veh_id = child.get("id")
            i_depart = int(float(child.get("depart")) / time_interval)
            i_arrival = int(float(child.get("arrival")) / time_interval)

            for trip in child:
                route_str = trip.get("edges")
                route = route_str.split(" ")
            edge_0 = route[0]
            edge_1 = route[-1]

            ele = edge_measure[edge_0]
            ele.enter_num[i_depart] += 1

            ele = edge_measure[edge_1]
            ele.leave_num[i_arrival] += 1

    return edge_measure


