
# link_info_cls: link information
# class link_info: id, from_node, to_node, length, freespeed, capacity, permlanes, oneway, modes, origid, type
class link_info_cls:
    def __init__(self, id, from_node, to_node, length, freespeed, capacity, permlanes, oneway, modes, origid, type):
        self.id = id
        self.from_node = from_node
        self.to_node = to_node
        self.length = float(length)
        self.freespeed = float(freespeed)
        self.capacity = float(capacity)
        self.permlanes = permlanes
        self.oneway = oneway
        self.modes = modes
        self.origid = origid
        self.type = type

# cls event stores information of events. In main it stores only sumo-area-related events (sumo_events)
class event:
    def __init__(self, veh, ty, tm, lk, pos):
        self.veh_id = veh
        self.type = ty
        self.time = tm
        self.link = lk
        self.pos = pos

# cls event_ele transfers information of events from line segments read from event.xml
class event_ele:
    def __init__(self, segments):
        i = 0
        # if there is "relativePosition" value, self.pos would be updated rather than -1
        self.pos = -1
        for ele in segments:
            i += 1
            if i == 1:
                self.time = float(ele[14:])
                continue
            if i == len(segments):
                continue
            seg2 = ele.split('=')
            tag = seg2[0]
            value = seg2[1]
            value = value[1:]

            if tag == "type":
                self.type = value
            if tag == "person" or tag == "vehicle":
                self.id = value
            if tag == "link":
                self.link = value
            if tag == "relativePosition":
                self.pos = float(value)
            if tag == "time":
                self.time = float(value)

# cls stores measurements of link
class link_measurement_ele:
    def __init__(self, link_id, length, n):
        self.link_id = link_id
        self.length = length
        self.edge = ''
        # queue = { veh_id: [veh1, enter_time1] }
        self.queue = {}
        self.enter_num = [0] * n
        self.leave_num = [0] * n
        self.avg_speed = [0] * n

class edgemeasure_ele:
    def __init__(self, edge_id, length):
        self.edge_id = edge_id
        self.length = length
        self.enter_num = []
        self.leave_num = []
        self.queue = []
        self.avg_speed = []
        self.travelTime = []
        self.timeLoss = []

class compare_ele:
    def __init__(self, link_info, edge_info, time_interval, scale):
        self.link_id = link_info.link_id
        self.edge_id = edge_info.edge_id
        self.link_len = link_info.length
        self.edge_len = edge_info.length
        self.comp_enter = []
        self.comp_leave = []
        self.comp_speed = []
        self.comp_travelTime = []
        self.comp_travelTime_per_meter = []
        self.comp_flow = []

        for i in range(len(link_info.enter_num)):
            self.comp_enter.append(int(link_info.enter_num[i]) * 100 / scale - int(edge_info.enter_num[i]))
            self.comp_leave.append(int(link_info.leave_num[i]) * 100 / scale - int(edge_info.leave_num[i]))

            if float(link_info.avg_speed[i]) != 0 and float(edge_info.avg_speed[i]) != 0:
                self.comp_travelTime.append(float(link_info.length) / float(link_info.avg_speed[i]) * int(link_info.leave_num[i]) - float(edge_info.length) / float(edge_info.avg_speed[i]))
                self.comp_speed.append(float(link_info.avg_speed[i]) / int(link_info.leave_num[i]) - float(edge_info.avg_speed[i]))
            else:
                self.comp_travelTime.append(0)
                self.comp_speed.append(0)

            if float(edge_info.avg_speed[i]) != 0 and float(link_info.avg_speed[i]) != 0:
                self.comp_travelTime_per_meter.append(int(link_info.leave_num[i]) / float(link_info.avg_speed[i]) - 1 / float(edge_info.avg_speed[i]))
            else:
                self.comp_travelTime_per_meter.append(0)
            self.comp_flow.append((int(link_info.leave_num[i]) * 100 / scale - int(edge_info.leave_num[i])) / time_interval * 3600)

class link_variant:
    def __init__(self, link_id, time_interval, link_info):
        self.link = link_id
        self.n_interval = int(86400 / time_interval)
        self.capacity = [link_info.capacity] * self.n_interval
        self.freespeed = [link_info.freespeed] * self.n_interval

