# This script is to extract events, which happen inside the SUMO-area, and which types are only "vehicle enters traffic,
# entered link, left link, vehicle leaves traffic" 4 types.
# Important: each line printed should end with " />", there is a space before / (to make the mf.matsim_output_trans_line more easy)

import main as mf
import argparse
import time
import gzip
import ClassDefine as cd

def matsim_event_extract_line(file, matsim_link_sumo):
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

        if type == "entered link":
            link_id = child.link
            if link_id in SUMO_LINK:
                one_event = cd.event(veh_id, type, child.time, link_id, child.pos)
                sumo_events.append(one_event)

        if type == "vehicle leaves traffic" or type == "left link":
            link_id = child.link
            if link_id in SUMO_LINK:
                one_event = cd.event(veh_id, type, child.time, link_id, child.pos)
                sumo_events.append( one_event )

    endt = time.time()
    print(f"MATSim output event file transfered, time='{endt-start}'")

    return sumo_events


def write_sumo_events(file, sumo_events):
    f = open(file, 'w')

    f.write('<?xml version="1.0" encoding="UTF-8"?>\n<events version="1.0">\n')
    for event in sumo_events:
        type = event.type
        if type == "vehicle enters traffic":
            print(f' <event time="{event.time}" type="vehicle enters traffic" link="{event.link}" vehicle="{event.veh_id}" networkMode="car" relativePosition="{event.pos}" />', file=f)
        if type == "entered link":
            print(f' <event time="{event.time}" type="entered link" link="{event.link}" vehicle="{event.veh_id}" />', file=f)
        if type == "left link":
            print(f' <event time="{event.time}" type="left link" link="{event.link}" vehicle="{event.veh_id}" />', file=f)
        if type == "vehicle leaves traffic":
            print(f' <event time="{event.time}" type="vehicle leaves traffic" person="{event.veh_id}" link="{event.link}" vehicle="{event.veh_id}" networkMode="car" relativePosition="{event.pos}" />', file=f)

    print('</events>', file=f)

    f.close()

parser = argparse.ArgumentParser()
parser.add_argument('--file_path', help='events_file path', type=str)
parser.add_argument('--link_file_path', help='path to link-edge transform file', type=str)
parser.add_argument('--name', help='Project Name', type=str)
parser.add_argument('--scale', help='Scale factor in %, only for Munich scenario', type=int)
parser.add_argument('--iter', help='Iteration number', type=int)
args = parser.parse_args()

files = mf.configs( args )
events = "scalingSFExp0.75CFExp1TEST_2016.output_events.xml.gz"

SUMO_LINK = []
SUMO_EDGE = []
LINK_EDGE_DIC = {}

files = mf.configs( args )

link_edge_file = "./scenario/" + files.name + "/link_to_edge.txt"
events_file = './scenario/' + files.name + '/' + events
events_output_file ='./scenario/' + files.name + '/' + str(args.scale) + '_sumo_' + events[0:-3]
matsim_map_path = files.matsim_add + '/scenarios/Munich/' + files.matsim_map

[SUMO_LINK, SUMO_EDGE, LINK_EDGE_DIC] = mf.load_link_edge(link_edge_file)
matsim_node, matsim_link, matsim_link_sumo = mf.load_link(mf.parse_xml_gz(matsim_map_path))
sumo_events = matsim_event_extract_line(events_file, matsim_link_sumo)
write_sumo_events(events_output_file, sumo_events)
