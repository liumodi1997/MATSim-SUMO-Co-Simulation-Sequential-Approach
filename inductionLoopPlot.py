import matplotlib.pyplot as plt
import numpy as np
import xml.etree.ElementTree as ET
from scipy import signal

# Load the induction loop measure data and transfer into np_array
def load_data(file):
    tree = ET.parse(file)
    root = tree.getroot()
    k = 0
    begin_time = 0
    end_time = 0
    loop_index = {}
    n_loop = 0
    last_n_loop = -1
    loop_nVeh = []
    loop_flow = []
    loop_speed = []
    time = [0]

    for child in root:
        if child.tag == "interval":
            t_begin = float(child.get('begin'))
            t_end = float(child.get('end'))
            id = child.get('id')
            nVehContrib = int(child.get('nVehContrib'))
            flow = float(child.get('flow'))
            speed = float(child.get('speed'))
            k += 1

            if last_n_loop == n_loop:
                if k % n_loop == 2:
                    time.append(t_begin)

            last_n_loop = n_loop
            if id not in loop_index:
                loop_index[id] = n_loop
                loop_nVeh.append([])
                loop_flow.append([])
                loop_speed.append([])
                n_loop += 1

            index = loop_index[id]
            loop_nVeh[index].append(nVehContrib)
            loop_flow[index].append(flow)
            loop_speed[index].append(speed)

    np_loop_nVeh = np.array(loop_nVeh)
    np_loop_flow = np.array(loop_flow)
    np_loop_speed = np.array(loop_speed)
    return time, np_loop_nVeh, np_loop_flow, np_loop_speed

def filtering(arr, bb, aa):
    return (signal.filtfilt(bb, aa, arr))

# Plot 3 types of measurements in one figure, each subplot includes 1 measure of the same road, but in different factors
# E.g., on road 0: nVeh_factor_0 + nVeh_factor_10 + nVeh_factor_20
def plot_figure(nVeh, flow, speed, time, loop_id, factor_list, wn, plot_id):
    fig = plt.figure(num=loop_id, figsize=(18, 4))
    #fig.title(f"speed + vehsPerHour*factor, at induction loop #'{loop_id}', filtered with wn='{wn}'")
    length = len(nVeh)

    ax1 = fig.add_subplot(131)
    ax1.set_xlabel("time/s")
    ax1.set_ylabel("Number of Veh")
    for i in range(length):
        ax1.plot(time, nVeh[i], label=factor_list[i])
    ax1.legend(loc=2, labelspacing=1, handlelength=3, fontsize=8, shadow=True)
    ax1.set_title(f"nVeh, filtered")

    ax2 = fig.add_subplot(132)
    ax2.set_ylabel("Flow veh/h")
    for i in range(length):
        ax2.plot(time, flow[i], label=factor_list[i])
    ax2.legend(loc=2, labelspacing=1, handlelength=3, fontsize=8, shadow=True)
    ax2.set_title("Flow, filtered")

    ax3 = fig.add_subplot(133)
    ax3.set_ylabel("speed m/s")
    for i in range(length):
        ax3.plot(time, speed[i], label=factor_list[i])
    ax3.legend(loc=2, labelspacing=1, handlelength=3, fontsize=8, shadow=True)
    ax3.set_title("Average speed, filtered")
    plt.show()

if __name__ == '__main__':
    time, np_loop_nVeh, np_loop_flow, np_loop_speed = load_data('speed_veh_1.xml')
    time, np_loop_nVeh2, np_loop_flow2, np_loop_speed2 = load_data('speed_veh_10.xml')
    time, np_loop_nVeh3, np_loop_flow3, np_loop_speed3 = load_data('speed_veh_20.xml')

    # Parameters defined by user
    # disp_list: which induction loop measurements are displayed
    disp_list = [0, 25, 53]
    # factor_list: which factor multiplied to VehsPerHour in calibrator
    factor_list = [1, 10, 20]
    # wn: filter parameter
    wn = 200 / 1000

    f_nVeh = []
    f_flow = []
    f_speed = []
    f_nVeh2 = []
    f_flow2 = []
    f_speed2 = []
    f_nVeh3 = []
    f_flow3 = []
    f_speed3 = []

    bb, aa = signal.butter(4, wn, 'low')
    for i in disp_list:
        f_nVeh.append(filtering(np_loop_nVeh[i], bb, aa))
        f_nVeh2.append(filtering(np_loop_nVeh2[i], bb, aa))
        f_nVeh3.append(filtering(np_loop_nVeh3[i], bb, aa))

        f_flow.append(filtering(np_loop_flow[i], bb, aa))
        f_flow2.append(filtering(np_loop_flow2[i], bb, aa))
        f_flow3.append(filtering(np_loop_flow3[i], bb, aa))

        f_speed.append(filtering(np_loop_speed[i], bb, aa))
        f_speed2.append(filtering(np_loop_speed2[i], bb, aa))
        f_speed3.append(filtering(np_loop_speed3[i], bb, aa))


    # plt.plot(time,np_loop_nVeh,linestyle='--',color='green',marker='*')
    k = 0
    for i in range(len(disp_list)):
        plot_figure([f_nVeh[i], f_nVeh2[i], f_nVeh3[i]], [f_flow[i], f_flow2[i], f_flow3[i]], [f_speed[i], f_speed2[i], f_speed3[i]], time, disp_list[i], factor_list, wn, i)
        k += 1




