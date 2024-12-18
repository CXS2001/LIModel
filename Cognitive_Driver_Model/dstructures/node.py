import copy
import itertools

import Cognitive_Driver_Model.utils.globalvalues as gv
from Cognitive_Driver_Model.utils.extendmath import d_distance_of_lon_action
from Cognitive_Driver_Model.envs.world import CarModule
from Cognitive_Driver_Model.envs.world import EgoModule



class Node(CarModule):
    """
    节点: 包含一个场景的驾驶环境
    """

    def __init__(self, ego_id, main_id, map, virtual_vehicle_dict) -> None:
        super().__init__(ego_id, main_id, map)
        self._virtual_vehicle_dict = virtual_vehicle_dict  # 一般只考虑距离较近的车辆


class Leaf(Node):
    """
    叶子节点: 下一时刻的驾驶环境
    """

    def __init__(self, ego_id, main_id, map, virtual_vehicle_dict) -> None:
        super().__init__(ego_id, main_id, map, virtual_vehicle_dict)


    def clone_self(self):
        """
        弥补deepcopy无法作用于carla中的某些数据类型
        """
        new_vvehicle_dict = {}
        for vid in self._virtual_vehicle_dict.keys():
            new_vvehicle_dict[vid] = self._virtual_vehicle_dict[vid].clone_self()
        return Leaf(
            self._ego_id,
            self._main_id,
            self._map,
            new_vvehicle_dict,
        )



class EgoNode(EgoModule):
    def __init__(self, ego_id, map, virtual_ego_vehicle) -> None:
        super().__init__(ego_id, map)
        self._virtual_ego_vehicle = virtual_ego_vehicle
        # 记录父亲编号
        self._father = None
        # 记录是否错过目标
        self._tag = 0.0
        # 记录迭代后的风险值
        self._risk = 0.0

    def clone_self(self):
        new_virtual_ego_vehicle = self._virtual_ego_vehicle.clone_self()
        return EgoNode(
            self._ego_id,
            self._map,
            new_virtual_ego_vehicle,
        )

    def generate_ego_leaves(self):
        """
        遍历ego的动作列表并生成下一时刻此动作对应的leaf,返回生成的ego_leaves列表
        """
        ego_leaves = []
        virtual_ego_vehicle = self._virtual_ego_vehicle
        consider_actions = ["MAINTAIN", "ACCELERATE", "DECELERATE"]
        if virtual_ego_vehicle._waypoint.lane_id != gv.LANE_ID["Left"]:
            consider_actions.append("SLIDE_LEFT")
        if virtual_ego_vehicle._waypoint.lane_id != gv.LANE_ID["Right"]:
            consider_actions.append("SLIDE_RIGHT")

        for v_ego_vehicle_action in consider_actions:
            # 生成下一时刻的ego车辆
            virtual_ego_vehicle_next = (
                self.generate_next_step_virtual_ego_vehicle(virtual_ego_vehicle, v_ego_vehicle_action)
            )
            ego_leaves.append(
                EgoNode(self._ego_id, self._map, virtual_ego_vehicle_next)
            )
        
        return ego_leaves
    
    def generate_next_step_virtual_ego_vehicle(self, v_ego_vehicle, v_ego_vehicle_action):
        """
        生成下一时刻的virtual ego vehicle
        """
        virtual_ego_vehicle_next = copy.copy(v_ego_vehicle)
        virtual_ego_vehicle_next._control_action = v_ego_vehicle_action

        if v_ego_vehicle_action in ["MAINTAIN", "ACCELERATE", "DECELERATE"]:
            # 需要计算离散时间下ego的前进距离
            d_distance = d_distance_of_lon_action(v_ego_vehicle, v_ego_vehicle_action, 1)
            virtual_ego_vehicle_next._waypoint = v_ego_vehicle._waypoint.next(d_distance)[0]

        if v_ego_vehicle_action == "SLIDE_LEFT":
            d_distance = max(v_ego_vehicle._scalar_velocity * 1, 1e-9)
            # 左侧道路上的路点
            virtual_ego_vehicle_next._waypoint = v_ego_vehicle._waypoint.next(d_distance)[0].get_left_lane()

        if v_ego_vehicle_action == "SLIDE_RIGHT":
            d_distance = max(v_ego_vehicle._scalar_velocity * 1, 1e-9)
            virtual_ego_vehicle_next._waypoint = v_ego_vehicle._waypoint.next(d_distance)[0].get_right_lane()

        virtual_ego_vehicle_next._transform = virtual_ego_vehicle_next._waypoint.transform
        virtual_ego_vehicle_next._scalar_velocity = (
            v_ego_vehicle._scalar_velocity + gv.LON_ACC_DICT.get(v_ego_vehicle_action)
        )
        return virtual_ego_vehicle_next



class CIPORoot(Node):
    """
    根节点: 包含当前时间驾驶环境
    """

    def __init__(
        self, ego_id, main_id, map, virtual_vehicle_dict, lon_levels, lat_levels
    ) -> None:
        super().__init__(ego_id, main_id, map, virtual_vehicle_dict)

        self._lon_levels = lon_levels
        self._lat_levels = lat_levels

    def clone_self(self):
        """
        弥补deepcopy无法作用于carla中的某些数据类型
        """
        new_vvehicle_dict = {}
        for vid in self._virtual_vehicle_dict.keys():
            new_vvehicle_dict[vid] = self._virtual_vehicle_dict[vid].clone_self()
        return CIPORoot(
            self._ego_id,
            self._main_id,
            self._map,
            new_vvehicle_dict,
            self._lon_levels,
            self._lat_levels,
        )

    def generate_leaves(self, dir_type):
        """
        生成叶子节点
        dir_type = "longitude" 或 "lateral"
        返回所有叶子节点与其数量
        """
        leaves = []
        iter_list = self.generate_iter_list(
            dir_type
        )  # dir_type决定叶子结点是纵向还是横向
        v_vehicle_id_list = list(self._virtual_vehicle_dict.keys())
        # 遍历每一个action的组合
        for comb in iter_list:
            virtual_vehicle_dict_leaf = {}
            for i in range(len(comb)):
                # 此处保证了v_action与vid是一一对应的
                v_action = comb[i]
                vid = v_vehicle_id_list[i]
                v_vehicle = self._virtual_vehicle_dict.get(vid)
                virtual_vehicle_dict_leaf[vid] = (
                    self.generate_next_step_virtual_vehicle(v_vehicle, v_action)
                )
            leaves.append(
                Leaf(self._ego_id, self._main_id, self._map, virtual_vehicle_dict_leaf)
            )
        return leaves, len(leaves)

    def generate_iter_list(self, dir_type):
        """
        由于每个车辆所考虑的动作空间不同, 该方法可以返回不定长的动作空间的排列组合。
        dir_type = "longitude" 或 "lateral"
        """
        res = []
        consider_actions = []
        if not self._virtual_vehicle_dict:
            return res
        # 纵向动作
        if dir_type == "longitude":
            for vid in self._virtual_vehicle_dict.keys():
                v_vehicle = self._virtual_vehicle_dict.get(vid)
                # 自身
                if vid == self._ego_id:
                    consider_actions = ["MAINTAIN", "ACCELERATE", "DECELERATE"]
                if vid in self._lon_levels["Level1"]:
                    consider_actions = ["MAINTAIN", "DECELERATE"]
                if vid in self._lon_levels["Level2"]:
                    # 两车道限制
                    if v_vehicle._waypoint.lane_id == gv.LANE_ID["Left"]:
                        consider_actions = ["MAINTAIN", "SLIDE_RIGHT"]
                    if v_vehicle._waypoint.lane_id == gv.LANE_ID["Right"]:
                        consider_actions = ["MAINTAIN", "SLIDE_LEFT"]
                if vid in self._lon_levels["Level3"]:
                    consider_actions = ["MAINTAIN"]
                res.append(consider_actions)
        # 横向动作
        if dir_type == "lateral":
            for vid in self._virtual_vehicle_dict.keys():
                v_vehicle = self._virtual_vehicle_dict.get(vid)
                if vid == self._ego_id:
                    # 两车道限制
                    if v_vehicle._waypoint.lane_id == gv.LANE_ID["Left"]:
                        consider_actions = ["SLIDE_RIGHT"]
                    if v_vehicle._waypoint.lane_id == gv.LANE_ID["Right"]:
                        consider_actions = ["SLIDE_LEFT"]
                if vid == self._lat_levels["Level1"][0]:
                    # 在主车的侧前方
                    consider_actions = ["MAINTAIN", "DECELERATE"]
                if vid == self._lat_levels["Level1"][1]:
                    # 在主车的侧后方
                    consider_actions = ["MAINTAIN", "ACCELERATE"]
                if vid in self._lat_levels["Level2"]:
                    # 两车道限制
                    if v_vehicle._waypoint.lane_id == gv.LANE_ID["Left"]:
                        consider_actions = ["MAINTAIN", "SLIDE_RIGHT"]
                    if v_vehicle._waypoint.lane_id == gv.LANE_ID["Right"]:
                        consider_actions = ["MAINTAIN", "SLIDE_LEFT"]
                if vid in self._lat_levels["Level3"]:
                    consider_actions = ["MAINTAIN"]
                res.append(consider_actions)
        # print(self._ego_id, list(itertools.product(*res)))
        return list(itertools.product(*res))

    def generate_next_step_virtual_vehicle(self, virtual_vehicle, control_action):
        """
        生成下一时刻的virtual vehicle
        """
        virtual_vehicle_next = copy.copy(virtual_vehicle)
        virtual_vehicle_next._control_action = control_action
        if control_action in ["MAINTAIN", "ACCELERATE", "DECELERATE"]:
            # 需要计算离散时间下的前进距离
            d_distance = d_distance_of_lon_action(virtual_vehicle, control_action, 1)
            virtual_vehicle_next._waypoint = virtual_vehicle._waypoint.next(d_distance)[
                0
            ]
        if control_action == "SLIDE_LEFT":
            d_distance = max(virtual_vehicle._scalar_velocity * 1, 1e-9)
            # 左侧道路上的路点
            virtual_vehicle_next._waypoint = virtual_vehicle._waypoint.next(d_distance)[
                0
            ].get_left_lane()
        if control_action == "SLIDE_RIGHT":
            d_distance = max(virtual_vehicle._scalar_velocity * 1, 1e-9)
            # 右侧道路上的路点
            virtual_vehicle_next._waypoint = virtual_vehicle._waypoint.next(d_distance)[
                0
            ].get_right_lane()
        virtual_vehicle_next._transform = virtual_vehicle_next._waypoint.transform
        virtual_vehicle_next._scalar_velocity = (
            virtual_vehicle._scalar_velocity + gv.LON_ACC_DICT.get(control_action)
        )
        return virtual_vehicle_next
 
