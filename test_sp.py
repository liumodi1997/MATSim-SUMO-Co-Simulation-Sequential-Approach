import argparse
import xml.etree.ElementTree as ET
import main as mf
NUM_VEH = 2000
SUMO_LINK = []
SUMO_EDGE = []
LINK_EDGE_DIC = {}

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--file_path',help='events_file path',type=str)
    parser.add_argument('--link_file_path',help='path to link-edge transform file',type=str)
    parser.add_argument('--name',help='Project Name',type=str)
    args = parser.parse_args()
    files = mf.load_config( args.name )
    events_file = '.\\scenario\\' + files['name'] + '\\' + files['event_file']
    link_file = '.\\scenario\\' + files['name'] + '\\' + files['link_file']
    print('Events file path = ',events_file)
    print('Link-edge transform file path = ',link_file)

    [SUMO_LINK, SUMO_EDGE, LINK_EDGE_DIC] = mf.load_link_edge(link_file)

    root = mf.parse_xml_gz(events_file)
    veh = mf.matsim_output_trans(root)
    sumo_map_file = '.\\scenario\\' + files['name'] + '\\sumo\\' + files['sumo_map']
    mf.load_edge_info( sumo_map_file )

    for i in range(5):
        print("Veh = ",i,veh[i])
    routes = mf.find_cross_border(veh)
    routes = mf.edge_direction_check( routes )
    print(routes)
    route_file = '.\\scenario\\' + files['name'] + '\\sumo\\' + files['sumo_route']
    mf.sumo_files_gen(route_file,routes)
    mf.run_sumo(files)
    sumo_veh = mf.sumo_output_trans("vehroute.xml", len(routes))
    print("routes = ",len(routes),"sumo_veh = ",len(sumo_veh))

    time_gap = mf.cal_time_gap(routes, sumo_veh)

    mf.cal_score(time_gap)