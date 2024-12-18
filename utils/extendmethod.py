import copy
import json
import numpy as np
import math
from numpy import linalg as LA
from shapely.geometry import Polygon


def point2norm(point1, point2):
    """
    计算两个三维点之间的欧氏距离
    """
    if point1 is None or point2 is None:
        return float('inf')
    a = np.array([point1['x'], point1['y'], point1['z']])
    b = np.array([point2['x'], point2['y'], point2['z']])
    return LA.norm(a - b)

def polygon_point(polygonPointList):
    """
    将三维点列表转换为二维多边形表示
    """
    polygon = []
    for i in range(len(polygonPointList)):
        point = [polygonPointList[i]['x'], polygonPointList[i]['y']]
        polygon.append(point)
    return polygon

def position_rotate(position_in_vehicle, rotation_theta):
    """
    旋转二维位置
    """
    new_position = copy.deepcopy(position_in_vehicle)
    x = new_position[0]
    y = new_position[1]
    x1 = x * math.cos(rotation_theta) - y * math.sin(rotation_theta)
    y1 = x * math.sin(rotation_theta) + y * math.cos(rotation_theta)
    new_position[0] = x1
    new_position[1] = y1
    return new_position


def get_ego_polygon(ego_state):
    """
    计算车辆的四边形表示。
    """
    gps_offset = -1.348649
    ego_position = list(ego_state['pose']['position'].values())
    ego_length = ego_state['size']['length']
    ego_width = ego_state['size']['width']
    theta = ego_state['pose']['heading']
    front_left = [ego_length / 2 - gps_offset, ego_width / 2.0]
    front_right = [ego_length / 2.0 - gps_offset, -ego_width / 2.0]
    back_left = [-ego_length / 2.0 - gps_offset, ego_width / 2.0]
    back_right = [-ego_length / 2.0 - gps_offset, -ego_width / 2.0]
    poly1 = [position_rotate(front_left, theta)[0] + ego_position[0],
             position_rotate(front_left, theta)[1] + ego_position[1]]
    poly2 = [position_rotate(front_right, theta)[0] + ego_position[0],
             position_rotate(front_right, theta)[1] + ego_position[1]]
    poly3 = [position_rotate(back_right, theta)[0] + ego_position[0],
             position_rotate(back_right, theta)[1] + ego_position[1]]
    poly4 = [position_rotate(back_left, theta)[0] + ego_position[0],
             position_rotate(back_left, theta)[1] + ego_position[1]]
    ego_polygon = [poly1, poly2, poly3, poly4]
    return ego_polygon

def convert_velocity_to_speed(velocity):
    x = velocity.x
    y = velocity.y
    z = velocity.z
    return math.sqrt(x * x + y * y + z * z)

def carla_to_ros2(rotation):

    roll = math.radians(rotation.roll)
    pitch = -math.radians(rotation.pitch)
    yaw = -math.radians(rotation.yaw)
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
        yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
        yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)

    return qx, qy, qz, qw


def get_junction_ahead(waypoint):
        # return the distance to the junction ahead(max 200m), if no junction ahead, return -1
        waypoint_list = waypoint.next_until_lane_end(distance=200)

        junction_ahead = -1
        for wp in waypoint_list:
            if wp.is_junction:
                junction_ahead = get_distance_between_points(waypoint.transform, wp.transform)
                break

        return junction_ahead


def get_distance_between_points(first_wp, second_wp):
        # return the distance between two waypoint
        x1 = first_wp.location.x
        x2 = second_wp.location.x
        y1 = first_wp.location.y
        y2 = second_wp.location.y
        z1 = first_wp.location.z
        z2 = second_wp.location.z

        return pow(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2), 0.5)


def extract_characters(string):
    extracted_chars = ''
    for char in reversed(string):  # 从后向前遍历字符串中的每个字符
        if char.isalpha():  # 如果字符是字母，则停止提取
            break
        extracted_chars = char + extracted_chars  # 将字符添加到结果字符串的开头
    return extracted_chars

