import argparse
import xml.etree.ElementTree as ET
import main as mf
import csv
import Measurements as ms

NUM_VEH = 2000
SUMO_LINK = {}
SUMO_EDGE = {}
LINK_EDGE_DIC = {}

def count_veh(veh):
    num = 0
    for veh_id in veh:
        veh_info = veh[veh_id]
        for eve in veh_info:
            if eve[0] == '-1':
                num += 1
    print("counted num in veh(num of -1) = ", num)

def count_border(routes, out_k, out_2, in_k, in_1):
    out_end = 0 # car ends at border(does not leave)
    out_false = 0
    in_false = 0
    out_con = 0 # car turns around at border(leaves the edge)
    in_start = 0 # car starts at border(does not enter)
    in_con = 0 # car turns around at border(enters the edge)
    for route in routes:
        routing = route[1]
        for i in range(len(routing)):
            edge = routing[i]
            if edge == out_k:
                if i+1 < len(routing):
                    if routing[i + 1] == out_2:
                        out_con += 1
                    else:
                        out_false += 1
                else:
                    out_end += 1

            if edge == in_k:
                if i > 0:
                    if routing[i - 1] == in_1:
                        in_con += 1
                    else:
                        in_false += 1
                else:
                    in_start += 1
    print(f"Going out: out_end={out_end}, out_con={out_con}, out_false={out_false}")
    print(f"Going in: in_start={in_start}, in_con={in_con}, in_false={in_false}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--file_path', help='events_file path', type=str)
    parser.add_argument('--link_file_path', help='path to link-edge transform file', type=str)
    parser.add_argument('--name', help='Project Name', type=str)
    parser.add_argument('--scale', help='Scale factor in %, only for Munich scenario', type=int)
    parser.add_argument('--iter', help='Iteration number', type=int)
    args = parser.parse_args()

    files = mf.configs( args )
    matsim_map_path = files.matsim_add + '/scenarios/Munich/' + files.matsim_map
    #matsim_map_path = file.matsim_add + '/scenarios/Munich/studyNetworkDense.xml'
    events_file = './scenario/' + files.name + '/' + files.event_file
    link_file = './scenario/' + files.name + '/' + files.link_file
    print('Events file path = ', events_file)
    print('Link-edge transform file path = ', files.link_edge_file_path)
    link_modify = {}

    [SUMO_LINK, SUMO_EDGE, LINK_EDGE_DIC] = mf.load_link_edge(files.link_edge_file_path)
    matsim_node, matsim_link, matsim_link_sumo = mf.load_link(mf.parse_xml_gz(matsim_map_path))

    print(files.cali_file_path)

    root = mf.parse_xml(files.cali_file_path)
    cali_edge, cali_route = mf.load_cali(root)

    #root = mf.parse_xml_gz(events_file)
    veh, matsim_simu_time, sumo_events = mf.matsim_output_trans_line(events_file, matsim_link_sumo)
    sumo_map_file = './scenario/' + files.name + '/sumo/' + files.sumo_map
    sumo_edge = mf.load_edge_info( sumo_map_file )
    count_veh(veh)

    #for i in range(5):
    #    print("Veh = ", i, veh[i])

    routes = mf.find_cross_border(veh)

    # Count numbers of routes which take turns at border
    #count_border(routes, '89840554#0', '-89840554#1', '242358060#0', '-242358060#2')

    print(f"Before random, Number of routes = {len(routes)}")
    #routes = mf.edge_direction_check( routes )
    #routes = mf.check_routes(routes)
    #print(routes)

    link_measure, start_time_intervals = mf.matsim_measurements(files.interval, sumo_events, matsim_link_sumo)

    # Generate cars w.r.t normal distribution of depart time
    #routes = mf.random_cars_gen(routes, 50, 20)
    print(f"After random, Number of routes = {len(routes)}")

    mf.sumo_files_gen(files, routes, link_measure, start_time_intervals, cali_edge, cali_route, LINK_EDGE_DIC)
    mf.run_sumo(files)
    sumo_veh = mf.sumo_output_trans(files.vehroute_file_path, len(routes))
    print("routes = ", len(routes), "sumo_veh = ", len(sumo_veh))

    edge_measure = ms.load_edge_measurement(files.edgeMeasurement_file_path, sumo_edge, files.interval)
    edge_measure = ms.update_enter_leave(files.vehroute_file_path, edge_measure, files.interval)
    comp_measure = ms.compare_measurement(link_measure, edge_measure, LINK_EDGE_DIC, files.interval)

    time_gap = mf.cal_time_gap(routes, sumo_veh, files)

    mf.cal_score(time_gap)

    # Optimization & Event change file
    matsim_link_sumo = mf.matsim_network_opt(link_modify, matsim_link_sumo, link_measure, edge_measure, comp_measure,
                                             files.interval)
    event_change_tree = mf.create_event_change_file(files, link_modify)
    mf.update_event_change_file(files, event_change_tree, link_modify)