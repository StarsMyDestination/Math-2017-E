import numpy as np

def get_vehicle_info(vehicle_num):
    vehicle_type = vehicle_num % 12
    vehicle_type = int(vehicle_type / 3)
    if vehicle_type == 3:
        vehicle_type = 2
    vehicle_dnum = vehicle_num % 12 - 3 * vehicle_type
    return vehicle_type, vehicle_dnum


def compute_node_time(time_start_list, path_node_dict, runtime):
    node_time_dict = {}
    for i in range(24):
        node_time_list = []
        time_start = time_start_list[i]
        path = path_node_dict[i]
        vehicle_type, _ = get_vehicle_info(i)
        for node in path:
            node_time = time_start + runtime[path[0], node, vehicle_type]
            node_time_list.append(node_time)
        node_time_dict[i] = node_time_list
    return node_time_dict


def print_paths_name(path_node_dict, node_names, vehicle_names):
    for key, value in path_node_dict.items():
        print(vehicle_names[key], [node_names[x] for x in value])

def get_intersect(zone1, zone2):
    zone3 = [0, 0]
    if min(zone1[1], zone2[1])>max(zone1[0], zone2[0]):
        if zone1[1]>zone2[0]:
            zone3 = [max(zone1[0], zone2[0]), min(zone1[1], zone2[1])]
        if zone2[1]>zone1[0]:
            zone3 = [max(zone1[0], zone2[0]), min(zone1[1], zone2[1])]
    return zone3