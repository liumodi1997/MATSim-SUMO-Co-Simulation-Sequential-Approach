import argparse
import xml.etree.ElementTree as ET
import main as mf
NUM_VEH = 2000
SUMO_LINK = {}
SUMO_EDGE = {}
LINK_EDGE_DIC = {}

def count_veh(veh):
    num = 0
    for veh_info in veh:

        for eve in veh_info:
            if eve[0] == '-1':
                num += 1
    print("counted num in veh(num of -1) = ", num)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--file_path', help='events_file path', type=str)
    parser.add_argument('--link_file_path', help='path to link-edge transform file', type=str)
    parser.add_argument('--name', help='Project Name', type=str)
    args = parser.parse_args()

    files = mf.configs( args.name )
    matsim_map_path = files.matsim_add + '/scenarios/Munich/' + files.matsim_map
    #matsim_map_path = file.matsim_add + '/scenarios/Munich/studyNetworkDense.xml'
    events_file = './scenario/' + files.name + '/' + files.event_file
    link_file = './scenario/' + files.name + '/' + files.link_file
    print('Events file path = ', events_file)
    print('Link-edge transform file path = ', files.link_edge_file_path)

    [SUMO_LINK, SUMO_EDGE, LINK_EDGE_DIC] = mf.load_link_edge(files.link_edge_file_path)
    matsim_node, matsim_link, matsim_link_sumo = mf.load_link(mf.parse_xml_gz(matsim_map_path))

    print(files.cali_file_path)

    root = mf.parse_xml(files.cali_file_path)
    cali_edge, cali_route = mf.load_cali(root)

    #root = mf.parse_xml_gz(events_file)
    veh, matsim_simu_time, sumo_events = mf.matsim_output_trans_line(events_file, matsim_link_sumo)
    sumo_map_file = './scenario/' + files.name + '/sumo/' + files.sumo_map
    _ = mf.load_edge_info( sumo_map_file )
    count_veh(veh)

    #for i in range(5):
    #    print("Veh = ", i, veh[i])

    routes = mf.find_cross_border(veh)

    print(f"Before random, Number of routes = {len(routes)}")
    #routes = mf.edge_direction_check( routes )
    #routes = mf.check_routes(routes)
    #print(routes)

    link_measurements = mf.init_link_measurements(matsim_simu_time, files.interval)
    link_measurements, start_time_intervals = mf.matsim_measurements(matsim_simu_time, files.interval,
                                                                  link_measurements, sumo_events, matsim_link_sumo)

    # Generate cars w.r.t normal distribution of depart time
    #routes = mf.random_cars_gen(routes, 50, 20)
    print(f"After random, Number of routes = {len(routes)}")

    mf.sumo_files_gen(files, routes, link_measurements, start_time_intervals, cali_edge, cali_route)
    mf.run_sumo(files)
    sumo_veh = mf.sumo_output_trans(files.vehroute_file_path, len(routes))
    print("routes = ", len(routes), "sumo_veh = ", len(sumo_veh))

    time_gap = mf.cal_time_gap(routes, sumo_veh)

    mf.cal_score(time_gap)