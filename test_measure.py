import main as mf
import Measurements as ms
import csv
import gzip
import xml.etree.ElementTree as ET
import os


dir = os.getcwd()
scale = '100'
sumo_link, sumo_edge, link_edge_dic = mf.load_link_edge("./scenario/Leopoldstrasse/link_to_edge.txt")
matsim_node, matsim_link, matsim_link_sumo = mf.load_link(mf.parse_xml_gz("K:/LMD2/Project/MA/MATSim/scenarios/Munich/munich-v1.0-network.xml.gz"))
sumo_edge = mf.load_edge_info(dir + "/scenario/Leopoldstrasse/sumo/osm.net.xml")
sumo_event_file = dir + "/scenario/Leopoldstrasse/" + scale + "_sumo_scalingSFExp0.75CFExp1TEST_2016.output_events.xml"
sumo_events, start_time, end_time = ms.transfer_link(sumo_event_file, sumo_link)
edgeMeasurement_file = dir + '/scenario/Leopoldstrasse/sumo/EdgeMeasurement.xml'
rou_file = dir + '/scenario/Leopoldstrasse/sumo/sequential.rou.xml'
veh_file = dir + '/scenario/Leopoldstrasse/sumo/vehroute.xml'
time_interval = 900
link_measure, _ = mf.matsim_measurements(time_interval, sumo_events, matsim_link_sumo)
edge_measure = ms.load_edge_measurement(edgeMeasurement_file, sumo_edge, time_interval)
edge_measure = ms.update_enter_leave(veh_file, edge_measure, time_interval)
comp_measure = ms.compare_measurement(link_measure, edge_measure, link_edge_dic, time_interval)
link_csv_output = dir + "/scenario/Leopoldstrasse/Analysis/" + scale + "link_measurements_" + str(time_interval) + ".csv"
edge_csv_output = dir + "/scenario/Leopoldstrasse/Analysis/" + scale + "edge_measurements_" + str(time_interval) + ".csv"
comp_csv_output = dir + "/scenario/Leopoldstrasse/Analysis/" + scale + "comp_measurements_" + str(time_interval) + ".csv"
ms.create_csv(link_csv_output, edge_csv_output, comp_csv_output, link_measure, edge_measure, comp_measure, time_interval, 0, 86400)