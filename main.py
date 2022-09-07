# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


import subprocess as sp
import xml.etree.ElementTree as ET
import traci
import gzip

NUM_VEH = 1000
SUMO_LINK = []

#SUMO_EDGE = { edge_id(str): [ from_id, to_id ], ..., }
SUMO_EDGE = {}
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

def load_config(name):
    xml_file = ".\scenario\\" + name + '\\Config.xml'
    root = parse_xml(xml_file)
    files = {}
    for child in root:
        if child.tag == "event-file":
            files['event_file'] = child.get('value')
        if child.tag == "link-file":
            files['link_file'] = child.get('value')
        if child.tag == "sumo-map":
            files['sumo_map'] = child.get('value')
        if child.tag == "sumo-route":
            files['sumo_route'] = child.get('value')
        if child.tag == "sumo-output":
            files['sumo_output'] = child.get('value')
        if child.tag == "matsim":
            files['matsim_add'] = child.get('value')
        if child.tag == "matsim-output":
            files['matsim_output'] = child.get('value')
    files['name'] = name

    return files

def get_veh_index( id ):
    return veh_dict.get( id )

def link_to_edge(link):
    return LINK_EDGE_DIC[ link ]

def load_link_edge(file):
    f = open(file,'r')

    #line = "$link $edge1 $edge2 ... $edgeN\n"
    line = f.readline()
    while line != '':

        #segments = [ 'link', 'edge1', 'edge2', ..., 'edgeN\n' ]
        segments = line.split(' ')

        #edges = [ 'edge1', 'edge2', ..., 'edgeN\n' ]
        for i in range(len(segments)):
            if i == 0:
                link = segments[i]
                SUMO_LINK.append(link)
                edges = []
            else:
                edges.append(segments[i].replace("\n",''))

        LINK_EDGE_DIC[ link ] = edges
        line = f.readline()

    f.close()
    return SUMO_LINK,SUMO_EDGE,LINK_EDGE_DIC



def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.

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

def matsim_output_trans(root):

    #veh[i] = [  [link_1,time_1(depart_time)], [link_2,time_2], ..., [link_k,time_k], [-1, arr_time] ]
    veh = []

    num_veh = 0
    for child in root:
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
    for child in root:
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
            event = [child.get("link"),float(child.get("time"))]
            veh[ get_veh_index(veh_id) ].append(event)

        #arrival time and flag "-1"
        if type == "vehicle leaves traffic":
            event = ["-1",float(child.get("time"))]
            veh[ get_veh_index(veh_id) ].append(event)

    return veh

def takeFirst(elem):
    return elem[0]

def find_cross_border(veh):
    #routes = [[depart_time, [L1,L2,...Ln] in EDGE as routing, arr_time] as route,[],...]

    #routes = [ [route_0], [route_1] , ... ]
    #route = [ depart_time, [routing] ,  arrival_time ]
    #routing = [ 'L1', 'L2', ... ]
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

def load_edge_info( xml_file ):
    root = parse_xml( xml_file )
    for child in root:
        if child.tag == 'edge' and child.get( 'function' ) != 'internal':
            edge_id = child.get( 'id' )
            edge = [ child.get( 'from' ), child.get( 'to' ) ]
            SUMO_EDGE[ edge_id ] = edge

def edge_direction_check( routes ):
    for route in routes:
        routing = route[1]
        for i in range( 1, len( routing ) ):
            [ i_from_id, i_to_id ] = SUMO_EDGE.get( routing[i] )


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

def sumo_files_gen(route_file,routes):
    rf = open(route_file,'w+')
    rf.write('<?xml version="1.0" encoding="UTF-8"?>\n')
    rf.write('<routes>\n')
    rf.write('   <vType accel="3.0" decel="6.0" id="CarA" length="5.0" minGap="2.5" maxSpeed="50.0" sigma="0.5" />\n\n')
    id = 1
    for route in routes:
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

def run_sumo(files):
    traci.start(["sumo", "-c", ".\\scenario\\" + files['name'] +  "\\sumo\\" + "sequential.sumocfg", "--vehroute-output", "vehroute.xml"])
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
            veh_id = int(child.get("id"))
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
    for i in range(len(routes)):
        time_gap[i] = [ routes[i][0] - sumo_veh[i][1] , routes[i][-1] - sumo_veh[i][2] ,\
                        (routes[i][-1] - routes[i][0]) - (sumo_veh[i][2] - sumo_veh[i][1])]
    return time_gap

def cal_score(time_gap):
    id = 1
    for ele in time_gap:
        print(id, ele)
        id += 1

#def output_files():

def main_iter( files ):
    score = 1
    score_thres = 0
    while score > score_thres:
        #jar_path = 'K:\LMD2\Project\MA\MATSim\matsim-example-project-0.0.1-SNAPSHOT.jar'
        jar_path = files['matsim_add'] + '\\' + 'matsim-example-project-0.0.1-SNAPSHOT.jar'
        #config_path = 'K:\LMD2\Project\MA\MATSim\scenarios\straight1\config.xml'
        config_path = files['matsim_add'] + '\\' + 'scenarios\\' + files['name'] + '\\' + 'config.xml'
        #run_matsim_jpype(jar_path, config_path)
        run_matsim_subprocess(jar_path, config_path)

        #load  output files from output_events.xml
        matsim_output_events_file = files['matsim_add'] + '\\' + 'scenarios\\' + files['name'] + '\\' + files['matsim_output']
        xml_root = parse_xml_gz(matsim_output_events_file)

        #transfer tree data to route table
        veh = matsim_output_trans(xml_root)

        veh_cross = find_cross_border(veh)
        num_veh_cross = len(veh_cross)

        #log_cross_border_time_matsim()

        route_file = files['sumo_route']
        sumo_files_gen(route_file,veh_cross)

        run_sumo()

        sumo_veh = sumo_output_trans(files['sumo_output'],num_veh_cross)

        time_gap = cal_time_gap(veh_cross,sumo_veh)

        cal_score(time_gap)

        score = 0
    #output_files()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')
    files = load_config("MaxVorstadt")
    #files = [event_file, link_file, matsim_map, sumo_route, sumo_output, matsim_add, matsim_output]
    """Set jPype environments and start up JVM
    jvm_path = jp.getDefaultJVMPath()
    jp.startJVM(jvm_path)
    jp.shutdownJVM()"""

    main_iter( files )







# See PyCharm help at https://www.jetbrains.com/help/pycharm/
