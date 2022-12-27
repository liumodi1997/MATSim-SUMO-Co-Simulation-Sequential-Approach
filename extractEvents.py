# This script is to extract events, which happen inside the SUMO-area, and which types are only "vehicle enters traffic,
# entered link, left link, vehicle leaves traffic" 4 types.
# Important: each line printed should end with " />", there is a space before / (to make the mf.matsim_output_trans_line more easy)

import main as mf
import argparse

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
veh, matsim_simu_time, sumo_events = mf.matsim_output_trans_line(events_file, matsim_link_sumo)
write_sumo_events(events_output_file, sumo_events)
