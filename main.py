# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


import subprocess as sp
import xml.etree.ElementTree as ET
import traci
import gzip

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
veh_dict = {}

def parse_xml_gz(xml_file):
    input = gzip.open(xml_file,'r')
    tree = ET.parse(input)
    root = tree.getroot()
    return root

def parse_xml(xml_file):
    tree = ET.parse(xml_file)
    root = tree.getroot()
    return root

class configs:
    def __init__(self, name):
        xml_file = ".\scenario\\" + name + '\\Config.xml'
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

        self.name = name
        self.jar_path = self.matsim_add + '\\' + 'matsim-example-project-0.0.1-SNAPSHOT.jar'
        self.matsim_config_path = self.matsim_add + '\\' + 'scenarios\\' + name + '\\' + 'config.xml'
        self.matsim_output_events_file_path = self.matsim_add + '\\' + 'scenarios\\' + name + '\\' + 'output\\output_events.xml.gz'
        self.route_file_path = '.\\scenario\\' + name + '\\sumo\\' + self.sumo_route
        self.add_file_path = '.\\scenario\\' + name + '\\sumo\\addition.xml'
        self.link_edge_file_path = '.\\scenario\\' + name + '\\' + self.link_file
        self.cali_file_path = '.\\scenario\\' + name + '\\' + self.cali

def get_veh_index(id):
    return veh_dict.get(id)

def link_to_edge(link):
    return LINK_EDGE_DIC[ link ]

def load_link_edge(file):
    print('Loading link-edge file\n')
    f = open(file,'r')

    #line = "$link $edge1 $edge2 ... $edgeN\n"
    line = f.readline()
    ilink = 0
    while line != '':

        #segments = [ 'link', 'edge1', 'edge2', ..., 'edgeN\n' ]
        segments = line.split(' ')

        #edges = [ 'edge1', 'edge2', ..., 'edgeN\n' ]
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
    return SUMO_LINK,SUMO_EDGE,LINK_EDGE_DIC



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
    instr_run_subprocess = ['java','-Xmx512m','-cp',jar_path,'org.matsim.core.controler.Controler',config_path]
    try:
        sp_run_matsim = sp.Popen(instr_run_subprocess,stderr=sp.PIPE)
    except sp.CalledProcessError as err:
        print('ERROR: ',err)
    else:
        #print('Returncode: ', instr_run_subprocess.returncode)
        print("0")


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
    cali_edge = []
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

class event:
    def __init__(self, veh, ty, tm, lk):
        self.veh_id = veh
        self.type = ty
        self.time = tm
        self.link = lk

def matsim_output_trans(root):
    print('Transforming MATSim output\n')

    #veh[i] = [  [link_1,time_1(depart_time)], [link_2,time_2], ..., [link_k,time_k], [-1, arr_time] ]
    veh = []
    num_event = 0

    num_veh = 0
    #count the number of vehicles and events
    for child in root:
        num_event += 1
        id1 = child.get("vehicle")
        id2 = child.get("person")
        if id1 is not None:
            veh_id = id1
        else:
            veh_id = id2

        '''if flag[veh_id]:
            #veh.append(event)  Wrong way to define!!!
            flag[veh_id] = 0
            num_veh = num_veh + 1'''
        if get_veh_index( veh_id ) is None:
            veh_dict[ veh_id ] = num_veh
            num_veh += 1
    print("num_veh = ", num_veh)

    veh = [[] for i in range(num_veh)]

    # build the list, whose index is the Vehicle_ID, and whose context is routes and time
    # record the start time and end time of all events
    # build the list of class event, which includes the events in SUMO-area
    sumo_events = []
    n = 0
    for child in root:
        n += 1
        if n == 1:
            start_time = float(child.get('time'))
        if n == num_event:
            end_time = float(child.get('time'))

        #depart time and starting lane
        type = child.get("type")

        #get the ID of vehcle OR person
        id1 = child.get("vehicle")
        id2 = child.get("person")
        if id1 is not None:
            veh_id = id1
        else:
            veh_id = id2

        if type == "vehicle enters traffic" or type == "entered link":
            event_list = [child.get("link"),float(child.get("time"))]
            veh[ get_veh_index(veh_id) ].append(event_list)

            link_id = child.get('link')
            if link_id in SUMO_LINK:
                one_event = event( veh_id, type, float(child.get('time')), link_id)
                sumo_events.append( one_event )


        #arrival time and flag "-1"
        if type == "vehicle leaves traffic":
            event_list = ["-1",float(child.get("time"))]
            veh[ get_veh_index(veh_id) ].append(event_list)

        if type == "vehicle leaves traffic" or type == "left link":
            link_id = child.get('link')
            if link_id in SUMO_LINK:
                one_event = event( veh_id, type, float(child.get('time')), link_id)
                sumo_events.append( one_event )
    time_dur = end_time - start_time

    return veh, time_dur, sumo_events

class measurement:
    def __init__(self, link_id, n_interval):
        self.id = link_id
        self.edge = ''
        #queue = [ [veh1, enter_time1], [veh2, enter_time2], ..., ]
        self.queue = []
        self.enter_num = [0] * n_interval
        self.leave_num = [0] * n_interval
        self.speed_average = [0] * n_interval
        self.len = 1

def init_link_measurements( matsim_simu_time, time_interval ):
    n_interval = int(matsim_simu_time / time_interval)
    link_measurements = []
    for link_in_sumo in SUMO_LINK:
        link_info = measurement( link_in_sumo, n_interval )
        link_measurements.append( link_info )
    return link_measurements

def matsim_measurements( matsim_simu_time, time_interval, link_measurements, sumo_events):
    print('Measuring MATSim traffic flows\n')
    start_time_intervals = []
    n_interval = matsim_simu_time / time_interval
    i_interval = 0
    i = 0

    for one_event in sumo_events:
        time = one_event.time

        #set start_time of the events;
        #initialise the 1st interval
        if i == 0:
            start_time = time
            start_time_intervals.append( start_time )
            i_interval += 1
            i += 1

        #if the time is larger than the end time of ith-interval, then the next interval will be considered
        while time > ( start_time + i_interval * time_interval ):
            start_time_intervals.append( start_time + i_interval * time_interval )
            i_interval += 1

        type = one_event.type

        # get the ID of vehcle OR person
        veh_id = one_event.veh_id

        if type == 'entered link' or type == "vehicle enters traffic":
            link_id = one_event.link
            if link_id in SUMO_LINK:
                link_index = SUMO_LINK[ link_id ]
                link_measurements[link_index].queue.append([veh_id, time])
                link_measurements[link_index].enter_num[i_interval] += 1

                # store the corresponding edge-id to the list, but only the 1st edge if one link to more edges
                edges = LINK_EDGE_DIC[link_id]
                link_measurements[link_index].edge = edges[0]
                edge1 = SUMO_EDGE[edges[0]]
                link_measurements[link_index].len = edge1[2]

        if type == 'left link' or type == 'vehicle leaves traffic':
            link_id = one_event.link
            if link_id in SUMO_LINK:
                link_index = SUMO_LINK[ link_id ]
                link_measurements[ link_index ].leave_num[ i_interval ] += 1

                #find the veh in queue, and read the enter time
                for veh_in_queue in link_measurements[ link_index ].queue:
                    if veh_in_queue[0] == veh_id:
                        enter_time = veh_in_queue[ 1 ]
                        break

                link_measurements[ link_index ].speed_average[i_interval] += ( time - enter_time ) / link_measurements[ link_index ].len

    return link_measurements,start_time_intervals

def takeFirst(elem):
    return elem[0]

def find_cross_border(veh):
    print('Finding vehicles which cross the SUMO borders')
    # routes = [[depart_time, [L1,L2,...Ln] in EDGE as routing, arr_time] as route,[],...]
    # if depart_time=-1: wrong route, ignore this agent

    # routes = [ [route_0], [route_1] , ... ]
    # route = [ depart_time, [routing] ,  arrival_time ]
    # routing = [ 'L1', 'L2', ... ]
    routes = []
    num_veh_sumo = 0
    for vehicle in veh:
        #flag is 0:this route not in SUMO
        flag = 0
        route = []
        routing = []
        for event in vehicle:
            link = event[0]

            #if border-crossing detected, then set the flag and start recording the route
            if link in SUMO_LINK:
                if flag == 0:
                    flag = 1
                    num_veh_sumo = num_veh_sumo + 1
                    route.append(event[1])

                if flag:

                    #Add the edge (transfered from link) to routing.
                    #Avoide the repeating edges and links. If more than 1 links refer to one edge, just ignore them
                    #If one link refers to more than 1 link, add them all
                    #But if there is no edge in routing yet, just add the edge directly
                    edges = link_to_edge(link)
                    if len( routing ) != 0:
                        if (routing[ -1 ] != edges[ 0 ]) :
                            for k in range(len( edges )):
                                routing.append( edges[k] )
                        else:
                            a= 0
                    else:
                        for k in range(len(edges)):
                            routing.append(edges[k])
            else:
                if (link == '-1') or (flag == 1):
                    flag = 0

                    #add routing and arrival time if this car ever runs in SUMO area
                    if len(route):
                        route.append(routing)
                        route.append(event[1])
                        routing = []
                if len(route):
                    routes.append(route)
                    route = []
    routes.sort(key=takeFirst)
    return routes

#def log_cross_border_time_matsim():

# read net.xml file, record each edge's start and end point, and length
def load_edge_info( xml_file ):
    root = parse_xml( xml_file )
    for child in root:
        if child.tag == 'edge' and child.get( 'function' ) != 'internal':
            edge_id = child.get( 'id' )
            shape = child.get("shape")
            if shape is None:
                for gchild in child:
                    shape = gchild.get('shape')

            shape = shape.split(' ') #shape = [ ['x1,y1'] ['x2,y2'] ... ]
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

def sumo_files_gen(route_file, add_file, routes, link_measurements, start_time_intervals, time_interval, cali_edge, cali_route):
    print('Generating SUMO input files\n')
    rf = open(route_file,'w+')
    rf.write('<?xml version="1.0" encoding="UTF-8"?>\n')
    rf.write('<routes>\n')
    rf.write('   <vType accel="3.0" decel="6.0" id="CarA" length="5.0" minGap="2.5" maxSpeed="50.0" sigma="0.5" />\n\n')
    id = 1
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

    # Generating additional file for calibrator
    af = open(add_file,'w+')
    af.write('<?xml version="1.0" encoding="UTF-8"?>\n')
    af.write('<additional>\n')
    m_id = 0
    for route_info in cali_route:
        print(f' <route id="{route_info[0]}" edges="{route_info[1]}"/>', file = af)


    for edge_info in cali_edge:

        edge_id = edge_info[0]
        edge_id_pos = edge_id
        if edge_id[0] == '-':
            edge_id_pos = edge_id[1:]
        for i in range(len(link_measurements)):
            measure = link_measurements[i]
            if measure.edge == edge_id_pos:
                link_index = i
                break

        if link_index == -1:
            print(edge_id)
        measure = link_measurements[link_index]
        print_flag = 0
        i = 0
        # Calculate the measurements and check if vehsPerHour==0
        for start_time in start_time_intervals:
            end_time = start_time + time_interval
            vph = int(measure.enter_num[i] / time_interval * 3600)
            if measure.leave_num[i] != 0:
                speed = measure.speed_average[i] / measure.leave_num[i]
            else:
                speed = -1
            if vph > 0 and speed > 0:
                print_flag = 1
            i += 1

        i = 0
        if print_flag:
            print(f' <calibrator id="{m_id}" edge="{edge_id}" pos="0" output="detector.xml">', file=af)
            m_id += 1
            for start_time in start_time_intervals:
                end_time = start_time + time_interval
                vph = int(measure.enter_num[i] / time_interval * 3600)
                if measure.leave_num[i] != 0:
                    speed = measure.speed_average[i] / measure.leave_num[i]
                else:
                    speed = -1
                if vph > 0 and speed > 0:
                    print_flag = 1
                i += 1
                if vph > 0 and speed > 0:
                    print(f'  <flow begin="{start_time}" end="{end_time}" vehsPerHour="{vph}" route="{edge_info[1]}"/> ', file=af)
                    #print(f'  <flow begin="{start_time}" end="{end_time}" vehsPerHour="300" speed="{speed}"/>',
                    #      file=af)
            print(f' </calibrator>\n', file=af)
    print(f'</additional>', file=af)
    af.close()

def run_sumo(files):
    traci.start(["sumo-gui", "-c", ".\\scenario\\" + files.name +  "\\sumo\\" + "sequential.sumocfg", "--vehroute-output", "vehroute.xml"])
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
    traci.close()

def sumo_output_trans(xml_file,num_veh_cross):
    root = parse_xml(xml_file)
    #sumo_veh = [ [ id, depart, arrival] , [] , ... , [] ]
    sumo_veh = [[] for _ in range(num_veh_cross)]
    id = 0
    for child in root:
        if child.tag == "vehicle":
            veh_id = child.get("id")
            try :
                veh_id = int(veh_id)
            except ValueError:
                continue
            depart = float(child.get("depart"))
            arrival = float(child.get("arrival"))
            sumo_veh[ veh_id - 1 ] = [ veh_id, depart, arrival ]
    return sumo_veh


def cal_time_gap(routes,sumo_veh):
    #time_gap = [ [ depart_time_gap , arrival_time_gap , used_time_gap ] , [] , ... , [] ]
    #time gap = Matsim_time - SUMO_time
    #gap > 0  ==>  MATSim run slow, arrive late
    time_gap = [[] for _ in range(len(routes))]
    i = 0
    count = 0
    for i in range(len(routes)):
        route = routes[i]
        if route[0] == -1:
            time_gap[i] = [ -999, -999, -999]
            count += 1
        else:
            time_gap[i] = [ routes[i][0] - sumo_veh[i - count][1] , routes[i][-1] - sumo_veh[i - count][2] ,\
                            (routes[i][-1] - routes[i][0]) - (sumo_veh[i - count][2] - sumo_veh[i - count][1])]
    return time_gap

def cal_score(time_gap):
    id = 1
    average = 0
    num = 0
    for ele in time_gap:
        print(id, ele)
        if abs(ele[1]) < 100:
            average += abs(ele[1])
            num += 1
        id += 1
    average /= num
    print(f"Average time gap = {average}")

#def output_files():

def main_iter( files ):
    score = 1
    score_thres = 0
    #jar_path = 'K:\LMD2\Project\MA\MATSim\matsim-example-project-0.0.1-SNAPSHOT.jar'

    #config_path = 'K:\LMD2\Project\MA\MATSim\scenarios\straight1\config.xml'


    time_interval = files.interval
    #run_matsim_jpype(jar_path, config_path)
    run_matsim_subprocess(files.jar_path, files.matsim_config_path)

    #load  output files from output_events.xml

    xml_root = parse_xml_gz(files.matsim_output_events_file)

    #transfer tree data to route table, and measure the traffic flows for calibrator
    veh, matsim_simu_time, sumo_events = matsim_output_trans(xml_root)
    link_measurements = init_link_measurements(matsim_simu_time, time_interval)
    link_measurements, start_time_intervals = matsim_measurements(xml_root, matsim_simu_time, time_interval, link_measurements, sumo_events)

    veh_cross = find_cross_border(veh)
    num_veh_cross = len(veh_cross)

    #log_cross_border_time_matsim()


    sumo_files_gen(files.route_file, files.add_file, veh_cross, link_measurements, start_time_intervals, time_interval)

    run_sumo(files)

    sumo_veh = sumo_output_trans(files['sumo_output'],num_veh_cross)

    time_gap = cal_time_gap(veh_cross,sumo_veh)

    cal_score(time_gap)

    score = 0
    #output_files()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    files = configs("MaxVorstadt")
    """Set jPype environments and start up JVM
    jvm_path = jp.getDefaultJVMPath()
    jp.startJVM(jvm_path)
    jp.shutdownJVM()"""

    main_iter( files )







# See PyCharm help at https://www.jetbrains.com/help/pycharm/
