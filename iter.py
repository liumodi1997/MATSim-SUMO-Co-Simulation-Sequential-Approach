import main as mf
import Measurements as ms
import argparse
import os

if __name__ == '__main__':
    # Step 1: Load files and configs
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', help='SUMO Area name', type=str)
    parser.add_argument('--scale', help='Scale factor in %, only for Munich scenario', type=int)
    parser.add_argument('--iter', help='Iteration number', type=int)
    args = parser.parse_args()

    files = mf.configs(args)
    [SUMO_LINK, SUMO_EDGE, LINK_EDGE_DIC] = mf.load_link_edge(files.link_edge_file_path)
    matsim_node, matsim_link, matsim_link_sumo = mf.load_link(mf.parse_xml(files.matsim_map_path))
    cali_edge, cali_route = mf.load_cali(mf.parse_xml(files.cali_file_path))
    sumo_edge = mf.load_edge_info(files.sumo_map_path)

    score = []
    iter_num = 0
    link_modify = {}
    average_gap = []

    # Iteration start
    while iter_num < files.max_iter:
        print("<*******************************************************************************************************>")
        print("<********************************Iteration", iter_num, "*****************************************>")
        print("<*******************************************************************************************************>")
        iter_num += 1

        # Initialise event change file, and folder
        if iter_num == 1:
            event_change_tree = mf.create_event_change_file(files, {})
        dir = './scenario/' + files.name + '/Analysis/Scale_' + str(args.scale) + '_Iteration_' + str(iter_num - 1)
        if not os.path.exists(dir):
            os.makedirs(dir)


        # Step 2: MATSim Simulation and measurement
        print(mf.run_matsim_cmd(files.jar_path, files.matsim_config_path))
        veh, matsim_simu_time, sumo_events = mf.matsim_output_trans_line(files.event_file_path, matsim_link_sumo)
        link_measure, start_time_intervals = mf.matsim_measurements(files.interval, sumo_events, matsim_link_sumo)

        # Step 3: Demand transfer
        routes = mf.find_cross_border(veh)

        # Step 4: Insert cars and Write SUMO input files.
        routes = mf.random_cars_gen(routes, files.gauss_sigma, int(100 / args.scale))
        mf.sumo_files_gen(files, routes, link_measure, start_time_intervals, cali_edge, cali_route, LINK_EDGE_DIC)

        # Step 5: SUMO Simulation
        mf.run_sumo(files)

        # Step 6: Measurement and comparision
        sumo_veh = mf.sumo_output_trans(files.vehroute_file_path, len(routes))
        edge_measure = ms.load_edge_measurement(files.edgeMeasurement_file_path, sumo_edge, files.interval)
        edge_measure = ms.update_enter_leave(files.vehroute_file_path, edge_measure, files.interval)
        comp_measure = ms.compare_measurement(link_measure, edge_measure, LINK_EDGE_DIC, files.interval)
        gap_file = dir + '/TripGap_S' + str(args.scale) + '_I' + str(iter_num - 1) +  ".csv"
        time_gap = mf.cal_time_gap(routes, sumo_veh, gap_file)
        average_gap.append(mf.cal_score(time_gap))

        # Step 7: Optimization in MATSim network
        link_modify = mf.matsim_network_opt(link_modify, matsim_link_sumo, link_measure, edge_measure, comp_measure, files.interval)

        # Step 8: Write MATSim input file
        event_change_log = dir + "/eventChange_S" + str(args.scale) + "_I" + str(iter_num - 1) + ".xml"
        mf.update_event_change_file(files, event_change_tree, link_modify, event_change_log)

        # Step 9: Write data and log

        ending = "_measurements_T" + str(files.interval) + '_S' + str(args.scale) + '_I' + str(iter_num - 1) +  ".csv"
        link_csv_output = dir + "/link" + ending
        edge_csv_output = dir + "/edge" + ending
        comp_csv_output = dir + "/comp" + ending

        ms.create_csv(link_csv_output, edge_csv_output, comp_csv_output, link_measure, edge_measure, comp_measure,files.interval, 0, 86400)

    print('**********************************************Iteration Finish***************************************************')
    print('Average Gap:', average_gap)
