import main as mf
import Measurements as ms
import argparse
import csv
import gzip
import xml.etree.ElementTree as ET
import os

parser = argparse.ArgumentParser()
parser.add_argument('--file_path', help='events_file path', type=str)
parser.add_argument('--link_file_path', help='path to link-edge transform file', type=str)
parser.add_argument('--name', help='Project Name', type=str)
parser.add_argument('--scale', help='Scale factor in %, only for Munich scenario', type=int)
parser.add_argument('--iter', help='Iteration number', type=int)
args = parser.parse_args()

files = mf.configs( args )

dir = os.getcwd()
scale = args.scale
sumo_link, sumo_edge, link_edge_dic = mf.load_link_edge(files.link_edge_file_path)
matsim_map_path = './scenario/' + args.name + '/' +files.matsim_map
matsim_node, matsim_link, matsim_link_sumo = mf.load_link(mf.parse_xml_gz(matsim_map_path))
sumo_edge = mf.load_edge_info(dir + "/scenario/" + args.name + "/sumo/osm.net.xml")
sumo_event_file = dir + "/scenario/" + args.name + "/" + str(args.scale) + "_sumo_scalingSFExp0.75CFExp1TEST_2016.output_events.xml"
sumo_events, start_time, end_time = ms.transfer_link(sumo_event_file, sumo_link)
edgeMeasurement_file = dir + '/scenario/' + args.name + '/sumo/EdgeMeasurement.xml'
rou_file = dir + '/scenario/' + args.name + '/sumo/sequential.rou.xml'
veh_file = dir + '/scenario/' + args.name + '/sumo/vehroute.xml'
time_interval = 900
scale = str(args.scale)
link_measure, _ = mf.matsim_measurements(time_interval, sumo_events, matsim_link_sumo)
edge_measure = ms.load_edge_measurement(edgeMeasurement_file, sumo_edge, time_interval)
edge_measure = ms.update_enter_leave(veh_file, edge_measure, time_interval)
comp_measure = ms.compare_measurement(link_measure, edge_measure, link_edge_dic, time_interval, args.scale)
link_csv_output = dir + "/scenario/" + args.name + "/Analysis/" + scale + "link_measurements_" + str(time_interval) + ".csv"
edge_csv_output = dir + "/scenario/" + args.name + "/Analysis/" + scale + "edge_measurements_" + str(time_interval) + ".csv"
comp_csv_output = dir + "/scenario/" + args.name + "/Analysis/" + scale + "comp_measurements_" + str(time_interval) + ".csv"
ms.create_csv(link_csv_output, edge_csv_output, comp_csv_output, link_measure, edge_measure, comp_measure, time_interval, 0, 86400)