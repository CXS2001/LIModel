from Cognitive_Driver_Model.utils.globalvalues import OBSERVE_DISTANCE
from utils.extendmethod import get_distance_between_points
from Cognitive_Driver_Model.utils.extendmath import cal_distance_along_road, cal_rel_location_curve
from Cognitive_Driver_Model.envs.world import CarModule


class Observer(CarModule):
    """
    感知器：用于获取已知环境信息。
    """

    def __init__(self, ego_id, main_id, map) -> None:
        super().__init__(ego_id, main_id, map)
        self._close_vehicle_id_list = []

    def judge_if_close_to_ego(self, full_vehicle_id_dict, other_vehicle_id):
        """判断是否距离自身较近"""
        other_wp = self._map.get_waypoint(
            full_vehicle_id_dict[other_vehicle_id]._vehicle.get_location()
        )
        ego_wp = self._map.get_waypoint(
            full_vehicle_id_dict[self._ego_id]._vehicle.get_location()
        )
        
        lat_distance = get_distance_between_points(ego_wp, other_wp)
        # lat_distance = cal_distance_along_road(ego_wp, other_wp)
        if abs(lat_distance) <= OBSERVE_DISTANCE:
            return True
        else:
            return False


    def get_close_vehicle_id_list(self, full_vehicle_id_dict):
        """
        筛选距离较近的车辆id
        """
        res = []
        for vehicle_id in full_vehicle_id_dict.keys():
            if self.judge_if_close_to_ego(full_vehicle_id_dict, vehicle_id):
                res.append(vehicle_id)
        return res
    
    
    def if_dest_in_front(self, full_vehicle_id_dict, dest_wp):
        ego_location = full_vehicle_id_dict[self._ego_id]._vehicle.get_location()
        ego_wp = self._map.get_waypoint(ego_location)
        # ego朝向坐标
        ego_forward_vector = ego_wp.transform.get_forward_vector()
        # dest相对ego坐标，由ego指向dest
        relative_vector = dest_wp.transform.location - ego_location
        # 计算向量的点积
        dot_product = ego_forward_vector.dot(relative_vector)
        # 如果点积大于0，则目的地在ego的前方
        if dot_product > 0:
            return True
        else:
            return False


class FullObserver(Observer):
    """全观测"""

    def __init__(self, ego_id, main_id, map) -> None:
        super().__init__(ego_id, main_id, map)

    def observe(self, full_vehicle_id_dict):
        self._close_vehicle_id_list = self.get_close_vehicle_id_list(
            full_vehicle_id_dict
        )
        return self._close_vehicle_id_list


class CIPO_Observer(Observer):
    """有等级的观测"""

    def __init__(self, ego_id, main_id, map) -> None:
        super().__init__(ego_id, main_id, map)
        # 横向与纵向分别两种不同的规则
        self._lon_levels = {"Level1": [None], "Level2": [], "Level3": []}
        self._lat_levels = {"Level1": [None, None], "Level2": [], "Level3": []}
        

    def observe(self, full_vehicle_id_dict):
        """返回: 较近车辆序列, 纵向CIPO序列, 横向CIPO序列"""
        self.get_cipo_vehicle_id_dict(full_vehicle_id_dict)
        return self._close_vehicle_id_list, self._lon_levels, self._lat_levels
        
    def get_cipo_vehicle_id_dict(self, full_vehicle_id_dict):
        """筛选周围车辆的CIPO等级"""
        self._close_vehicle_id_list = []
        self._lon_levels = {"Level1": [None], "Level2": [], "Level3": []}
        self._lat_levels = {
            "Level1": [None, None],
            "Level2": [],
            "Level3": [],
        }  # 此处Level1分前后两种情况，提前分配空间
        # 首先筛选Level1
        min_dhw_lon, min_dhw_lat_pos = self.get_leve1_vehicle_id_list(
            full_vehicle_id_dict
        )
        # 然后筛选Level2和Level3
        self.get_remain_levels_vehicle_id_list(
            min_dhw_lon, min_dhw_lat_pos, full_vehicle_id_dict
        )

    def get_leve1_vehicle_id_list(self, full_vehicle_id_dict):
        """首先筛选一轮level1的车辆, 方便后续等级的车辆筛选"""
        min_dhw_lat_pos = 1e9
        min_dhw_lat_neg = 1e9
        min_dhw_lon = 1e9  # 前方最近车辆的距离
        self._close_vehicle_id_list = self.get_close_vehicle_id_list(
            full_vehicle_id_dict
        )
        for vehicle_id in self._close_vehicle_id_list:
            vehicle_wp = self._map.get_waypoint(
                full_vehicle_id_dict[vehicle_id]._vehicle.get_location()
            )
            ego_wp = self._map.get_waypoint(
                full_vehicle_id_dict[self._ego_id]._vehicle.get_location()
            )
            # vehicle相对ego的沿线距离
            lat_distance = cal_distance_along_road(ego_wp, vehicle_wp)
            # 对于即将由纵向动作生成的Node, 选取当前车道前方最近的车辆
            if (
                vehicle_wp.lane_id == ego_wp.lane_id
                and lat_distance > 0
                and lat_distance < min_dhw_lon
            ):
                self._lon_levels["Level1"][0] = vehicle_id
                min_dhw_lon = lat_distance
            # 对于即将由横向动作生成的Node, 选取相邻车道前后方的车辆
            if abs(vehicle_wp.lane_id - ego_wp.lane_id) == 1:
                # 选取相邻车道前方最近车辆
                if lat_distance >= 0 and lat_distance < min_dhw_lat_pos:
                    self._lat_levels["Level1"][0] = vehicle_id
                    min_dhw_lat_pos = lat_distance
                # 选取相邻车道后方最近车辆
                if lat_distance < 0 and -lat_distance < min_dhw_lat_neg:
                    self._lat_levels["Level1"][1] = vehicle_id
                    min_dhw_lat_neg = lat_distance
        return min_dhw_lon, min_dhw_lat_pos

    def get_remain_levels_vehicle_id_list(
        self, min_dhw_lon, min_dhw_lat_pos, full_vehicle_id_dict
    ):
        """筛选剩余level车辆"""
        for vehicle_id in self._close_vehicle_id_list:
            vehicle_wp = self._map.get_waypoint(
                full_vehicle_id_dict[vehicle_id]._vehicle.get_location()
            )
            ego_wp = self._map.get_waypoint(
                full_vehicle_id_dict[self._ego_id]._vehicle.get_location()
            )
            # vehicle相对ego的沿线距离
            lat_distance = cal_distance_along_road(ego_wp, vehicle_wp)
            # 纵向Node的情况，选取相邻车道侧前方车辆
            if (
                vehicle_id not in self._lon_levels["Level1"]
                and vehicle_id != self._ego_id
            ):
                if (
                    abs(vehicle_wp.lane_id - ego_wp.lane_id) == 1
                    and lat_distance >= 0
                    and lat_distance <= min_dhw_lon
                ):
                    self._lon_levels["Level2"].append(vehicle_id)
                else:
                    self._lon_levels["Level3"].append(vehicle_id)
            # 横向Node的情况，选取同车道前方车辆
            if (
                vehicle_id not in self._lat_levels["Level1"]
                and vehicle_id != self._ego_id
            ):
                if (
                    vehicle_wp.lane_id == ego_wp.lane_id
                    and lat_distance >= 0
                    and lat_distance <= min_dhw_lat_pos
                ):
                    self._lat_levels["Level2"].append(vehicle_id)
                else:
                    self._lat_levels["Level3"].append(vehicle_id)
