import subprocess as sp
import xml.etree.ElementTree as ET
import traci
import gzip
import time
import numpy as np
import csv
import ClassDefine as cd
import os

NUM_VEH = 1000
# SUMO_LINK = { 'link0':0, 'link1':1, ..., 'linkj':j }
# all the link in SUMO area, referring the number of <class list> link_measurements
SUMO_LINK = {}

# SUMO_EDGE = { edge_id(str): [ from_id, to_id, length ], ..., }
SUMO_EDGE = {}
# LINK_EDGE_DIC = { link_id(str): [ edge1, edge2, ..., edgek] }
LINK_EDGE_DIC = {}

# We use dict as the index of list veh:
# veh_dict = { veh_id(str): index(int) }, index = 0 ... num_veh-1
#veh_dict = {}

def parse_xml_gz(xml_file):
    input = gzip.open(xml_file,'r')
    tree = ET.parse(input)
    root = tree.getroot()
    return root

def parse_xml(xml_file):
    if xml_file[-1] == 'z':
        input = gzip.open(xml_file, 'r')
    else:
        input = xml_file
    tree = ET.parse(input)
    root = tree.getroot()
    return root

class configs:
    def __init__(self, args):
        name = args.name
        xml_file = "./scenario/" + args.name + '/Config.xml'
        root = parse_xml(xml_file)

        for child in root:
            if child.tag == "event-file":
                self.event_file = child.get('value')
            if child.tag == "link-file":
                self.link_file = child.get('value')
            if child.tag == "sumo-map":
                self.sumo_map = child.get('value')
            if child.tag == "sumo-route":
                self.sumo_route = child.get('value')
            if child.tag == "sumo-output":
                self.sumo_output = child.get('value')
            if child.tag == "matsim":
                self.matsim_add = child.get('value')
            if child.tag == "matsim-output":
                self.matsim_output = child.get('value')
            if child.tag == "interval":
                self.interval = int(child.get('value'))
            if child.tag == "calibration":
                self.cali = child.get('value')
            if child.tag == "matsim-map":
                self.matsim_map = child.get('value')
            if child.tag == "route-probe":
                self.route_probe = int(child.get('value'))
            if child.tag == "calibration-flag":
                self.calibration_flag = int(child.get('value'))
            if child.tag == "calibration-flow-factor":
                self.calibration_flow_factor = int(child.get('value'))
            if child.tag == "calibration-speed":
                self.calibration_speed = int(child.get('value'))
            if child.tag == "gauss-sigma":
                self.gauss_sigma = float(child.get('value'))
            if child.tag == "max-iter":
                self.max_iter = int(child.get('value'))

        # MATSim related
        self.matsim_scenario = 'fine_' + str(args.scale) + '_' + str(args.iter) + 'it'
        self.jar_path = self.matsim_add + '/' + 'matsim-example-project-0.0.1-SNAPSHOT.jar'
        self.matsim_config_path = self.matsim_add + '/' + 'scenarios/' + self.matsim_scenario + '/' + 'configBase.xml'
        self.matsim_output_events_file_path = self.matsim_add + '/' + 'scenarios/' + self.matsim_scenario + '/' + 'output/output_events.xml.gz'
        self.matsim_map_path = self.matsim_add + '/scenarios/' + self.matsim_scenario + '/' + self.matsim_map
        self.event_file_path = self.matsim_add + '/' + 'scenarios/' + self.matsim_scenario + '/output/test.output_events.xml.gz'
        self.event_change_file_path = self.matsim_add + '/scenarios/' + self.matsim_scenario + '/networkChangeEvents.xml'
        # SUMO related
        self.name = name
        self.route_file_path = './scenario/' + name + '/sumo/' + self.sumo_route
        self.add_file_path = './scenario/' + name + '/sumo/addition.xml'
        self.link_edge_file_path = './scenario/' + name + '/' + self.link_file
        self.cali_file_path = './scenario/' + name + '/' + self.cali
        self.vehroute_file_path = './scenario/' + name + '/sumo/' + 'vehroute.xml'
        self.netstate_file_path = './scenario/' + name + '/Analysis/' + 'netstate.xml'
        self.sumo_map_path = './scenario/' + name + '/sumo/' + self.sumo_map
        # Analysis related
        self.edge_measurement_file_path = './scenario/' + name + '/Analysis/' + 'EdgeMeasurement.xml'
        self.gap_file_path = './scenario/' + name + '/Analysis/' + 'TripGap.csv'
        self.edgeMeasurement_file_path = './scenario/' + name + '/sumo/' + 'EdgeMeasurement.xml'

        print('MATSim events file: ', self.matsim_output_events_file_path)



def link_to_edge(link):
    return LINK_EDGE_DIC[ link ]

def load_link_edge(file):
    print('Loading link-edge file')
    start = time.time()
    f = open(file,'r')

    #line = "$link $edge1 $edge2 ... $edgeN\n"
    line = f.readline()
    ilink = 0
    while line != '':

        #segments = [ 'link', 'edge1', 'edge2', ..., 'edgeN\n' ]
        segments = line.split(' ')

        #edges = [ 'edge1', 'edge2', ..., 'edgeN\n' ] or [ 'null' ]
        for i in range(len(segments)):
            if i == 0:
                link = segments[i]
                SUMO_LINK[link] = ilink
                ilink += 1
                edges = []
            else:
                edges.append(segments[i].replace("\n",''))

        LINK_EDGE_DIC[ link ] = edges
        line = f.readline()

    f.close()
    endt = time.time()
    print(f"Link-edge file loaded, time='{endt-start}'")
    return SUMO_LINK,SUMO_EDGE,LINK_EDGE_DIC

# matsim_node = [ [ node_id, x, y] ]
# matsim_link = [ [ link(class) ] ]
# matsim_link_sumo = { link_id: link(class) }


def load_link(root):
    print("Loading links")
    start = time.time()
    matsim_node = []
    matsim_link = []
    matsim_link_sumo = {}
    for child in root:
        if child.tag == "nodes":
            for child_node in child:
                node_info = [child_node.get('id'), child_node.get('x'), child_node.get('y')]
                matsim_node.append(node_info)
        if child.tag == "links":
            for child_link in child:
                link_id = child_link.get('id')
                link_ele = cd.link_info_cls(link_id, child_link.get('from'), child_link.get('to'),\
                                   child_link.get('length'), child_link.get('freespeed'), child_link.get('capacity'), \
                                                 child_link.get('permlanes'), child_link.get('oneway'), child_link.get('modes'),\
                                                 child_link.get('origid'), child_link.get('type'))

                matsim_link.append(link_ele)
                if link_id in SUMO_LINK:
                    if link_id not in matsim_link_sumo:
                        matsim_link_sumo[link_id] = link_ele
    endt = time.time()
    print(f"MATSim Links loaded, time='{endt-start}'")
    return matsim_node, matsim_link, matsim_link_sumo
        #if child.tag == "link"

#call the run_matsim.py script as subprocess to start JVM with JPype
def run_matsim_jpype(jar_path, config_path):
    instr_run_subprocess = ['python','run_matsim.py','-j',jar_path,'-c',config_path]
    try:
        sp_run_matsim = sp.Popen(instr_run_subprocess,stderr=sp.PIPE)
    except sp.CalledProcessError as err:
        print('ERROR: ',err)
    else:
        print('Returncode: ', instr_run_subprocess.returncode)

#direct run jar package with subprocess without starting JVM
def run_matsim_subprocess(jar_path, config_path):
    print('Running MATSim')
    instr_run_subprocess = ['java','-Xmx5120m','-cp',jar_path,'org.matsim.core.controler.Controler',config_path]
    try:
        sp_run_matsim = sp.Popen(instr_run_subprocess,stderr=sp.PIPE)
    except sp.CalledProcessError as err:
        print('ERROR: ',err)
    else:
        print('Returncode: ', sp_run_matsim.returncode)
        print("0")

def run_matsim_cmd(jar_path, config_path):
    print('Running MATSim')
    instr_run_subprocess = 'java -Xmx5120m -cp ' + jar_path + ' org.matsim.core.controler.Controler ' + config_path
    d = os.system(instr_run_subprocess)
    return d


#Add a OR delete '-' char before the Edge(str).
#But if the reversed edge does not exist, return -1
def reverse_di( str ):
    if str[ 0 ] == '-':
        if SUMO_EDGE.get( str[ 1: ] ) is not None:
            return str[ 1: ]
        else:
            return -1
    else:
        if SUMO_EDGE.get( '-' + str ) is not None:
            return '-' + str
        else:
            return -1

def load_cali(root):
    print('Loading calibration file')
    #cali_edge = [ [edge_id, route] ]
    cali_edge = []
    #cali_routes = [ [route_id, "edge1 edges2 ..."] ]
    cali_routes = []
    for child in root:
        if child.tag == "route":
            route_id = child.get('id')
            edges = child.get('edges')
            r = [route_id, edges]
            cali_routes.append(r)

        if child.tag == "edge":
            edge_id = child.get('id')
            route = child.get('route')
            r = [edge_id, route]
            cali_edge.append(r)
    return cali_edge, cali_routes


def matsim_output_trans_line(file, matsim_link_sumo):
    print('Transforming MATSim output (read by line)')
    start = time.time()
    #veh = { veh_id: [[],[],[],...] }
    #veh[veh_id] = [ [link_1,timei_1(depart_time),timeo_1], [link_2,timei_2,timeo_2], ..., [link_k,timei_k,timeo_k], ['-1',-1,arr_time] ]
    veh = {}
    num_event = 0
    num_veh = 0
    num_trip = 0
    count = 0
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
        if n == 0:
            start_time = child.time
        if line[:9] == "</events>":
            end_time = child.time
        n += 1

        # Init for a new car in veh
        try:
            veh_info = veh[veh_id]
        except KeyError:
            veh[veh_id] = []
            num_veh += 1
            veh_info = veh[veh_id]

        if type == "entered link":
            link_id = child.link
            if link_id in SUMO_LINK:
                one_event = cd.event(veh_id, type, child.time, link_id, child.pos)
                sumo_events.append(one_event)
                event_list = [child.link, child.time, 0]

                # Need to check whether this event starts a new route
                #veh_info = veh[get_veh_index(veh_id)]
                if len(veh_info) == 0:
                    # if len==0 means a new route is started
                    num_trip += 1
                    veh_info.append(event_list)
                else:
                    last_act = veh_info[-1]

                    # If last event is '-1', then add the link to the list, as a new route begins
                    if last_act[0] == '-1':
                        num_trip += 1
                        veh_info.append(event_list)
                    else:
                        # If last link is connected with this link, and enter time has no big gap, then we just add the link to bottom
                        # If last link is not connected, then a new route begins
                        # If enter time gap between last link and this link is huge, then a new route
                        last_link = last_act[0]
                        if matsim_link_sumo[last_link].to_node == matsim_link_sumo[link_id].from_node and (event_list[1] - last_act[2]) < 10:
                            veh_info.append(event_list)
                        else:
                            #print(f"last trip not -1 but link not connected, or time gap is big: last_link={last_link}, link={link_id}")
                            count += 1
                            last_act = veh_info[-1]
                            ele = ['-1', -1, last_act[-1]]
                            veh_info.append(ele)
                            num_trip += 1
                            veh_info.append(event_list)


        # arrival time and flag "-1"
        if type == "vehicle leaves traffic":
            event_list = ["-1", -1, child.time]
            veh_info.append(event_list)

        if type == "vehicle leaves traffic" or type == "left link":
            link_id = child.link
            if link_id in SUMO_LINK:
                one_event = cd.event(veh_id, type, child.time, link_id, child.pos)
                sumo_events.append( one_event )
                if type == "left link":
                    # Update the left time for last link, if last link "exists"!
                    # len==0 means, last event is "veh enters traffic". don't need to do anything
                    if len(veh_info) != 0:
                        last_act = veh_info[-1]
                        # Only when last event is "entered link", we update left time
                        if last_act[0] != '-1':
                            last_act[-1] = child.time

    # Add -1 flag in each veh at the end
    for veh_id in veh:
        veh_info = veh[veh_id]
        last_act = veh_info[-1]
        if last_act[0] != '-1':
            event_list = ['-1', -1, last_act[-1]]
            veh_info.append(event_list)

    time_dur = end_time - start_time
    print("MATSim num_veh = ", num_veh)
    print("MATSim num_trip = ", num_trip)
    print("count = ", count)
    endt = time.time()
    print(f"MATSim output event file transfered, time='{endt-start}'")

    return veh, time_dur, sumo_events


def init_link_measurements( matsim_simu_time, time_interval ):
    n_interval = int(matsim_simu_time / time_interval) + 1
    link_measurements = []
    for link_in_sumo in SUMO_LINK:
        link_info = cd.link_measurement_ele( link_in_sumo, n_interval )
        link_measurements.append( link_info )
    return link_measurements

def matsim_measurements( time_interval, sumo_events, matsim_link_sumo):
    print('Measuring MATSim traffic flows')
    start = time.time()
    start_time_intervals = [0]
    start_time = 0
    n_interval = int(86400 / time_interval)
    # link_measure = { link_id: link_measurement_ele }
    link_measure = {}
    i_interval = 1
    print(f"length of sumo_event = '{len(sumo_events)}'")
    count = 0

    for one_event in sumo_events:
        count += 1

        time1 = one_event.time
        type = one_event.type
        veh_id = one_event.veh_id
        pos = one_event.pos
        link_id = one_event.link

        # Add the empty link ele to link_measure
        if link_id not in link_measure:
            link_length = matsim_link_sumo[link_id].length
            ele = cd.link_measurement_ele(link_id, link_length, n_interval)
            link_measure[link_id] = ele
        this_link_measure = link_measure[link_id]


        #if the time is larger than the end time of ith-interval, then the next interval will be considered
        while time1 > (start_time + i_interval * time_interval ):
            start_time_intervals.append( start_time + i_interval * time_interval )
            i_interval += 1

        if type == 'entered link' or type == "vehicle enters traffic":
            if link_id in SUMO_LINK:

                #link_index = SUMO_LINK[ link_id ]

                #link_measurements[link_index].queue.append([veh_id, time1, pos, type, link_id])
                this_link_measure.queue[veh_id] = [time1, pos, type, link_id]
                this_link_measure.enter_num[i_interval - 1] += 1

                # store the corresponding edge-id to the list, but only the 1st edge if one link to more edges
                #edges = LINK_EDGE_DIC[link_id]
                #this_link_measure.edge = edges[0]


        if type == 'left link' or type == 'vehicle leaves traffic':
            if link_id in SUMO_LINK:

                #find the veh in queue, and read the enter time
                try:
                    veh_in_queue = this_link_measure.queue[veh_id]
                    enter_time = veh_in_queue[ 0 ]
                    enter_pos = veh_in_queue[ 1 ]
                    last_type = veh_in_queue[ 2 ]
                    last_link = veh_in_queue[ 3 ]

                    if time1 - enter_time != 0:
                        this_link_measure.avg_speed[i_interval - 1] += this_link_measure.length / (time1 - enter_time)
                        this_link_measure.leave_num[i_interval - 1] += 1
                    else:
                        this_link_measure.leave_num[i_interval - 1] += 1
                    del this_link_measure.queue[veh_id]
                except KeyError:
                    continue

    # Initialise link measurements for links without a car run into
    for link_id in matsim_link_sumo:
        if link_id not in link_measure:
            link_length = matsim_link_sumo[link_id].length
            ele = cd.link_measurement_ele(link_id, link_length, n_interval)
            link_measure[link_id] = ele

    endt = time.time()
    print(f"Measurement complete, time='{endt-start}'")
    return link_measure, start_time_intervals

def takeFirst(elem):
    return elem[0]

def find_cross_border(veh):
    print('Finding vehicles which cross the SUMO borders')
    start = time.time()
    # routes = [[depart_time, [L1,L2,...Ln] in EDGE as routing, arr_time] as route,[],...]
    # if depart_time=-1: wrong route, ignore this agent

    # routes = [ [route_0], [route_1] , ... ]
    # route = [ depart_time, [routing] , [leave_time], arrival_time ]
    # routing = [ 'L1', 'L2', ... ] in edge_id
    # enter_time = [ time1, time2, ... ], if more edges: linear interpolation
    routes = []
    num_veh_sumo = 0
    for veh_id in veh:
        vehicle = veh[veh_id]
        #flag is 0:this route not in SUMO
        flag = 0
        route = []
        routing = []
        leave_time = []

        # event = [ link_i, enter_time_i, out_time_i ]
        for event in vehicle:
            link = event[0]

            #if border-crossing detected, then set the flag and start recording the route
            if link in SUMO_LINK:
                # flag == 0 means, a new trip begins
                if flag == 0:
                    flag = 1
                    num_veh_sumo = num_veh_sumo + 1

                    # record departure time
                    route.append(event[1])

                if flag:

                    # Add the edge (transfered from link) to routing.
                    # Avoide the repeating edges and links. If more than 1 links refer to one edge, just ignore them
                    # If one link refers to more than 1 link, add them all
                    # But if there is no edge in routing yet, just add the edge directly
                    # If LINK refers to NULL, then ignore it
                    edges = link_to_edge(link)
                    if len( routing ) != 0:
                        if (routing[ -1 ] != edges[ 0 ]) :
                            for k in range(len( edges )):
                                if edges[k] != 'null':
                                    routing.append( edges[k] )
                                    leave_time.append(event[1] + (event[2] - event[1]) / len(edges))
                        else:
                            a= 0
                    else:
                        for k in range(len(edges)):
                            if edges[k] != 'null':
                                routing.append(edges[k])
                                leave_time.append(event[1] + (event[2] - event[1]) / len(edges))
            else:
                if (link == '-1') or (flag == 1):
                    flag = 0

                    #add routing and arrival time if this car ever runs in SUMO area
                    if len(route):
                        route.append(routing)
                        route.append(leave_time)
                        route.append(event[2])
                        routing = []
                        leave_time = []
                if len(route):
                    routes.append(route)
                    route = []
    routes.sort(key=takeFirst)
    endt = time.time()
    print(f"Corssing-bound cars founded, time='{endt-start}' number of routes={len(routes)}")
    print("find corss bound, num_veh = ", num_veh_sumo)
    return routes

# read net.xml file, record each edge's start and end point, and length
def load_edge_info( xml_file ):
    print('Loading Edges')
    root = parse_xml( xml_file )
    for child in root:
        if child.tag == 'edge' and child.get( 'function' ) != 'internal':
            edge_id = child.get( 'id' )
            shape = child.get("shape")
            if shape is None:
                for gchild in child:
                    shape = gchild.get('shape')

            try:
                shape = shape.split(' ') #shape = [ ['x1,y1'] ['x2,y2'] ... ]
            except AttributeError:
                shape = []

            point_list = []
            for coord in shape:
                coord2 = coord.split(',')
                point_list.append(coord2)
            length = 0
            for i in range(len(point_list)):
                if i == 0:
                    continue
                coord0 = point_list[i-1]
                coord1 = point_list[i]
                x1 = float(coord0[0])
                y1 = float(coord0[1])
                x2 = float(coord1[0])
                y2 = float(coord1[1])
                length += ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) ** 0.5

            edge = [ child.get( 'from' ), child.get( 'to' ), length ]
            SUMO_EDGE[ edge_id ] = edge
    return SUMO_EDGE

def edge_direction_check( routes ):
    for route in routes:
        routing = route[1]
        for i in range( 1, len( routing ) ):
            [ i_from_id, i_to_id, length ] = SUMO_EDGE.get( routing[i] )


            #Eliminate the '-' char if there is one at the beginning of routing[i-1]
            '''if routing[ i-1 ][ 0 ] == '-':
                edge = SUMO_EDGE.get( routing[ i-1 ][ 1: ] )
            else:
                edge = SUMO_EDGE.get( routing[ i-1 ] )'''
            edge = SUMO_EDGE.get( routing[ i-1 ] )

            last_from_id = edge[ 0 ]
            last_to_id = edge[ 1 ]
            if i_to_id == last_to_id and reverse_di( routing[ i ] ) != -1:
                routing[ i ] = reverse_di( routing[ i ] )
            elif i_to_id == last_from_id and reverse_di( routing[ i ] ) != -1 and reverse_di( routing[ i-1 ] ) != -1:
                routing[ i ] = reverse_di( routing[ i ] )
                routing[ i-1 ] = reverse_di( routing[ i-1 ] )
            elif i_from_id == last_from_id and reverse_di( routing[ i-1 ] ) != -1:
                routing[ i-1 ] = reverse_di( routing[ i-1 ] )

    return routes

# Check if car runs into an Einbahnstrasse. If yes, delete it.
def check_routes(routes):
    for route in routes:
        routing = route[1]
        for i in range(len(routing)):
            if i == 0:
                continue
            edge0_info = SUMO_EDGE[routing[i - 1]]
            edge1_info = SUMO_EDGE[routing[i]]
            if edge0_info[1] != edge1_info[0]:
                route[0] = -1
                break
    return routes

def random_cars_gen(routes, gauss_sigma, factor):
    print("Inserting random cars...")
    new_routes = []
    n = 0

    # route = [ depart_time, [routing] , [leave_time], arrival_time ]
    for route in routes:
        mu = route[0]
        gauss_dis = np.random.normal(loc=mu, scale=gauss_sigma, size=(factor))
        n += 1
        new_routes.append(route)
        for i in range(factor):
            new_route = [gauss_dis[i], route[1], route[2], route[-1] + (gauss_dis[i] - mu)]
            new_routes.append(new_route)
            n += 1
        if n % 100000 == 0:
            print(n)

    new_routes.sort(key=takeFirst)
    return new_routes

def sumo_files_gen(files, routes, link_measurements, start_time_intervals, cali_edge, cali_route, link_edge_dic):
    print('Generating SUMO input files')
    route_file = files.route_file_path
    add_file = files.add_file_path
    time_interval = files.interval
    route_probe = files.route_probe
    calibration_flag = files.calibration_flag
    calibration_flow_factor = files.calibration_flow_factor
    calibration_speed = files.calibration_speed
    rf = open(route_file,'w+')
    rf.write('<?xml version="1.0" encoding="UTF-8"?>\n')
    rf.write('<routes>\n')
    rf.write('   <vType accel="3.0" decel="6.0" id="CarA" length="5.0" minGap="2.5" maxSpeed="50.0" sigma="0.5" />\n\n')
    id = 0
    for route in routes:
        if route[0] == -1:
            continue
        routing = route[1]
        print(f'   <vehicle id="{id}" type="CarA" depart="{route[0]}" >',file=rf)
        rf.write('      <route edges="')
        for edge_id in range( len(routing)-1 ):
            rf.write(routing[edge_id])
            rf.write(' ')
        print(routing[-1],'"/>\n',file=rf)
        rf.write('   </vehicle>\n')
        id = id + 1
    rf.write('</routes>')
    rf.close()
    print("SUMO num_veh = ", id+1)

    # Generating additional file for calibrator
    af = open(add_file,'w+')
    af.write('<?xml version="1.0" encoding="UTF-8"?>\n')
    af.write('<additional>\n')

    if route_probe:
        n = 0
        for edge_info in cali_edge:
            freq = time_interval
            print(f' <routeProbe id="{edge_info[0]}" edge="{edge_info[0]}" freq="{time_interval/10}" file="routeProbe_output.xml"/>', file=af)

    m_id = 0
    for route_info in cali_route:
        print(f' <route id="{route_info[0]}" edges="{route_info[1]}"/>', file = af)


    for edge_info in cali_edge:

        edge_id = edge_info[0] # edge_ID

        # Find corresbonding Link_ID in DIC
        link_id = ''
        for link in link_edge_dic:
            edges = link_edge_dic[link]
            for edge in edges:
                if edge_id == edge:
                    link_id = link
                    break

        measure = link_measurements[link_id]
        print_flag = 0
        i = 0
        # Calculate the measurements and check if vehsPerHour==0
        for start_time in start_time_intervals:
            end_time = start_time + time_interval
            vph = int(measure.enter_num[i] / time_interval * 3600)
            if measure.leave_num[i] != 0:
                speed = measure.avg_speed[i] / measure.leave_num[i]
            else:
                speed = -1
            if vph > 0 and speed > 0:
                print_flag = 1
            i += 1

        i = 0
        if calibration_flag == 0:
            print_flag = 0

        if print_flag:
            length = measure.length
            print(f' <calibrator id="{m_id}" edge="{edge_id}" pos="0" jamThreshold="0.05" output="detector.xml"', end='', file=af)
            if route_probe:
                print(f' routeProbe="{edge_info[0]}"', end='', file=af)
            print('>', file=af)
            m_id += 1
            for start_time in start_time_intervals:
                end_time = start_time + time_interval
                vph = int(measure.enter_num[i] / time_interval * 3600)
                enter = measure.enter_num[i]
                leave = measure.leave_num[i]
                if measure.leave_num[i] != 0:
                    speed = measure.avg_speed[i] / measure.leave_num[i]
                else:
                    speed = -1
                if vph > 0 and speed > 0:
                    print_flag = 1
                i += 1
                if vph > 0 and speed > 0:
                    print(f'  <flow begin="{start_time}" end="{end_time}"',end=' ', file=af)
                    if calibration_flow_factor != 0 :
                        print(f'vehsPerHour="{vph * calibration_flow_factor}"', end=' ', file=af)
                    if calibration_speed != 0:
                        print(f'speed="{speed * calibration_speed}"',end=' ', file=af)
                    print(f'route="{edge_info[1]}"/>', file=af)

            print(f' </calibrator>\n', file=af)

    # Induction loop detectors
    loop_id = 0
    for edge_info in cali_edge:
        edge_id = edge_info[0]
        info = SUMO_EDGE[edge_id]
        pos = info[2] / 2
        lane = edge_id + "_0"
        print(f'  <inductionLoop id="loop{loop_id}" lane="{lane}" pos="{pos}" freq="{time_interval}" file="inductionLoopOut.xml"/>', file=af)
        loop_id += 1

    print(f' <edgeData id="edge_measurement" file="EdgeMeasurement.xml" freq="{time_interval}"/>', file=af)

    print(f'</additional>', file=af)
    af.close()

def run_sumo(files):
    print('Running SUMO')
    traci.start(["sumo", "-c", "./scenario/" + files.name +  "/sumo/" + "osm.sumocfg"])
    #  "--netstate-dump", files.netstate_file_path
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
    traci.close()

def sumo_output_trans(xml_file,num_veh_cross):
    root = parse_xml(xml_file)
    #sumo_veh = { 'id' : [ id, depart, arrival, route, leave_time] }
    sumo_veh = {}
    id = 0
    for child in root:
        if child.tag == "vehicle":
            # Veh ID from MATSim is int S, inserted vehicle's ID is S.xx
            veh_id = child.get("id")
            depart = float(child.get("depart"))
            arrival = float(child.get("arrival"))
            for gchild in child:
                route = gchild.get('edges')
                edges = route.split(' ')
                leave = gchild.get('exitTimes')
                leave_time = leave.split(' ')
            sumo_veh[ veh_id ] = [ veh_id, depart, arrival, edges, leave_time]
    return sumo_veh


def cal_time_gap(routes,sumo_veh, file):
    #time_gap = { id : [ depart_time_gap , arrival_time_gap , used_time_gap ] }
    #time gap = Matsim_time - SUMO_time
    #gap > 0  ==>  MATSim run slow, arrive late
    time_gap = {}
    i = 0
    count = 0
    for i in range(len(routes)):
        route = routes[i]
        veh = sumo_veh[str(i)]
        time_gap[i] = [ route[0] - veh[1] , route[-1] - veh[2] ,(route[-1] - route[0]) - (veh[2] - veh[1])]

    # Write the time gap in CSV
    f = open(file, 'w+', encoding='UTF8', newline='')
    writer = csv.writer(f)
    # 'Veh_ID', 'Depart Time Gap', 'Arrival Time Gap', 'Travel Time Gap'
    header = ['Veh_ID', 'Depart Time Gap', 'Arrival Time Gap', 'Travel Time Gap']
    writer.writerow(header)

    for i in range(len(time_gap)):
        element = time_gap[i]
        line = []
        line.append(str(i))
        line.append(element[0])
        line.append(element[1])
        line.append(element[2])

        # If bug happens, print the information
        if element[-1] > 2000:
            route = routes[i]
            rou = route[1]
            leave_time = route[2]
            line.append('leave_link_time')
            for j in range(len(leave_time)):
                line.append(str(leave_time[j]) + ',' + rou[j])

            line.append('SUMO trip')
            sumo_info = sumo_veh[str(i)]
            edges = sumo_info[3]
            leave_time = sumo_info[4]
            line.append(sumo_info[1])
            #line.append(sumo_info[2])
            for j in range(len(edges)):
                line.append(leave_time[j] + ',' + edges[j])

        writer.writerow(line)
    f.close()

    return time_gap

def cal_score(time_gap):
    id = 1
    average = 0
    num = 0
    for veh_id in time_gap:
        ele = time_gap[veh_id]
        #print(id, ele)
        if abs(ele[1]) < 100:
            average += abs(ele[1])
            num += 1
        id += 1
    average /= num
    print(f"Average time gap = {average}")
    return average

def matsim_network_opt(link_modify, matsim_link_sumo, link_measure, edge_measure, comp_measure, time_interval):
    print('Optimizing MATSim network link flow capacity')
    # link_modify records the modifications of link param
    # link_modify = { link_ID: cls link_varient }

    # Initialisation of link_modify
    if link_modify == {}:
        for link in matsim_link_sumo:
            ele = cd.link_variant(link, time_interval, matsim_link_sumo[link])
            link_modify[link] = ele

    n_interval = int(86400 / time_interval)
    for link in link_measure:
        try:
            comp_measure_ele = comp_measure[link]
        except KeyError:
            continue
        comp_measure_speed = comp_measure_ele.comp_speed
        to_modify = link_modify[link]
        for i in range(n_interval):
            speed_rate = comp_measure_speed[i] / to_modify.freespeed[i]
            if speed_rate < -1:
                speed_rate = -1
            to_modify.capacity[i] = to_modify.capacity[i] * (1 - speed_rate * 0.2)
    return link_modify

def create_event_change_file(files, link_modify):
    root = ET.Element("networkChangeEvents")
    root.attrib = {"xmlns":             "http://www.matsim.org/files/dtd",
                   "xmlns:xsi":         "http://www.w3.org/2001/XMLSchema-instance",
                   "xsi:schemaLocation":"http://www.matsim.org/files/dtd http://www.matsim.org/files/dtd/networkChangeEvents.xsd"}
    for link in link_modify:
        link_info = link_modify[link]
        for i in range(link_info.n_interval):
            node = ET.SubElement(root, "networkChangeEvent")
            node.attrib = {"startTime": str(files.interval * (link_info.n_interval - 1))}
            sub_node = ET.SubElement(node, "link")
            sub_node.attrib = {"refId": link}
            sub_node_2 = ET.SubElement(node, "capacity")
            sub_node_2.attrib = {"type": "absolute", "value": str(link_info.capacity[i])}
    tree = ET.ElementTree(root)
    tree.write(files.event_change_file_path, encoding="utf-8", xml_declaration=True)
    return tree

def update_event_change_file(files, tree, link_modify):
    print("Updating network event change file for MATSim")
    nodes = tree.findall("networkChangeEvent")
    for link in link_modify:
        link_info =  link_modify[link]
        node_link = []
        # Find all nodes, with sub node having same link ID
        for node in nodes:
            for sub_node in node:
                if sub_node.tag == 'link':
                    if sub_node.get('refId') == link:
                        node_link.append(node)

        for i in range(link_info.n_interval):
            time = files.interval * (i)
            for node in node_link:
                if node.attrib['startTime'] == time:
                    for sub_node in node:
                        if sub_node.tag == 'capacity':
                            sub_node.set('value', str(link_info.capacity[i]))
    tree.write(files.event_change_file_path, encoding="utf-8", xml_declaration=True)





# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    files = configs("MaxVorstadt")
    """Set jPype environments and start up JVM
    jvm_path = jp.getDefaultJVMPath()
    jp.startJVM(jvm_path)
    jp.shutdownJVM()"""

