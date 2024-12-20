import numpy as np
import copy

from Cognitive_Driver_Model.dstructures.ellipse import EllipseGenerator
from Cognitive_Driver_Model.decisionmodels.CDM.observer import CIPO_Observer
from Cognitive_Driver_Model.envs.world import CarModule

import Cognitive_Driver_Model.utils.extendmath as emath
import Cognitive_Driver_Model.utils.globalvalues as gv



class Risk(CarModule):
    """
    计算risk的基类, 各种方法都可继承于此
    """

    def __init__(self, ego_id, main_id, map) -> None:
        super().__init__(ego_id, main_id, map)
        self._risks = []  # risks与leaves一一对应

    def get_preferred_leaves(self, leaves, driving_style_risk):
        """
        筛选叶子结点
        """
        if len(self._risks) == 0:
            raise ValueError("You have not calculate risks.")

        preferred_leaves = []
        other_leaves = []  # 备用, 现在用来debug

        for i in range(len(leaves)):
            if driving_style_risk >= self._risks[i]:
                preferred_leaves.append(leaves[i])
            else:
                other_leaves.append(leaves[i])
        # 如果所有叶子节点都被删除, 选择风险最低的一个
        if len(preferred_leaves) == 0:
            preferred_leaves.append(leaves[np.argmin(self._risks)])

        return preferred_leaves, other_leaves
    


class CDMRisk(Risk):
    """
    基于椭圆社会力、超速等因素
    """

    def __init__(self, ego_id, main_id, map) -> None:
        super().__init__(ego_id, main_id, map)


    def cal_risk_list(self, root, leaves):
        """
        计算risk列表, 与leaves对齐
        """
        self._risks = [0 for _ in range(len(leaves))]
        for i in range(len(leaves)):
            social_force, _ = self.cal_social_force_max(root, leaves[i])
            penalty = self.cal_penalty_risk(root, leaves[i])
            ttc_risk = self.cal_ttc_risk(leaves[i])
            self._risks[i] = social_force + penalty + ttc_risk

    
    def cal_ego_tree_risk(self, ego_tree, dest):

        risk_dict = {
            "MAINTAIN": -1,
            "ACCELERATE": -1,
            "DECELERATE": -1,
            "SLIDE_LEFT": -1,
            "SLIDE_RIGHT": -1,
        }

        # 首先对所有节点进行标记
        for depth in range(1, 5):
            for i in range(len(ego_tree[depth])):
                ego_location = ego_tree[depth][i]._virtual_ego_vehicle._transform.location
                ego_wp = ego_tree[depth][i]._virtual_ego_vehicle._waypoint
                ego_lane_id = ego_wp.lane_id
                forward_distance = ego_location.distance_squared_2d(dest.transform.location) # 纵向距离

                lane_distance = abs(ego_lane_id - dest.lane_id)
                # ego朝向坐标
                ego_forward_vector = ego_wp.transform.get_forward_vector()
                relative_vector = dest.transform.location - ego_location

                # 计算向量的点积
                dot_product = ego_forward_vector.dot(relative_vector)

                # 如果点积大于0，则目的地在ego的前方
                if (dot_product > 0) and (forward_distance > 25.0):
                    ego_tree[depth][i]._tag = 0
                else:
                    ego_tree[depth][i]._tag = lane_distance

                ego_tree[depth][i]._risk = ego_tree[depth][i]._tag
        # 进行风险迭代
        for depth in range(4, 1, -1):
            for i in range(len(ego_tree[depth])):
                ego_tree[depth - 1][ego_tree[depth][i]._father]._risk += ego_tree[depth][i]._risk
                
        for i in range(len(ego_tree[1])):
            risk_dict[ego_tree[1][i]._virtual_ego_vehicle._control_action] = ego_tree[1][i]._risk

        return risk_dict




    def cal_ttc_risk(self, leaf):
        """
        计算单个叶子结点的ttc风险
        """
        ego_v_vehicle = leaf._virtual_vehicle_dict.get(self._ego_id)
        min_vid_front, min_dist_front, min_vid_back, min_dist_back = (
            emath.get_lon_closest_vehicle(leaf._virtual_vehicle_dict, self._ego_id)
        )
        ttc_front = -1
        ttc_back = -1
        # 前方ttc
        if min_vid_front != None:
            ttc_front = max(min_dist_front - gv.CAR_LENGTH, 0) / (
                -leaf._virtual_vehicle_dict.get(min_vid_front)._scalar_velocity
                + ego_v_vehicle._scalar_velocity
                + 1e9
            )
        # 后方ttc
        if min_vid_back != None:
            ttc_back = min(min_dist_front + gv.CAR_LENGTH, 0) / (
                leaf._virtual_vehicle_dict.get(min_vid_front)._scalar_velocity
                - ego_v_vehicle._scalar_velocity
                - 1e9
            )
        return self.ttc_to_risk(ttc_front) + self.ttc_to_risk(ttc_back)

    def cal_social_force_sum(self, root, leaf):
        """
        计算单个叶子结点的社会力风险(累加版)
        """
        total_risk = 0

        # 计算椭圆参数
        c1_location = root._virtual_vehicle_dict.get(self._ego_id)._transform.location
        c2_location = leaf._virtual_vehicle_dict.get(self._ego_id)._transform.location

        c = emath.cal_length(
            emath.cal_rel_location_curve(self._map, c2_location, c1_location)
        )

        # 生成椭圆
        ellipse = EllipseGenerator(self._map, c1_location, c2_location, c)

        # 计算风险
        for vid in leaf._virtual_vehicle_dict.keys():
            if vid != self._ego_id:
                car_location = root._virtual_vehicle_dict.get(vid)._transform.location
                total_risk += ellipse.cal_risk_vector(car_location)
        return total_risk

    def cal_social_force_max(self, root, leaf):
        """
        计算单个叶子结点的社会力风险(max版)
        返回最大风险和对应的vid
        """
        max_risk = 0
        max_vid = None
        # 计算椭圆参数
        c1_location = root._virtual_vehicle_dict.get(self._ego_id)._transform.location
        c2_location = leaf._virtual_vehicle_dict.get(self._ego_id)._transform.location

        c = (
            emath.cal_length(
                emath.cal_rel_location_curve(self._map, c2_location, c1_location)
            )
            / 2
        )

        # 生成椭圆
        ellipse = EllipseGenerator(self._map, c1_location, c2_location, c)

        # 计算风险
        for vid in leaf._virtual_vehicle_dict.keys():
            if vid != self._ego_id:
                car_location = root._virtual_vehicle_dict.get(vid)._transform.location
                elli_risk = ellipse.cal_risk_vector(car_location)
                if elli_risk >= max_risk:
                    max_risk = elli_risk
                    max_vid = vid
        return max_risk, max_vid

    def cal_penalty_risk(self, root, leaf):
        """
        计算超速、变道、碰撞惩罚
        """
        overspeed_penalty = 0
        lanechange_penalty = 0
        collision_penalty = 0
        traj_accuracy = 6  # 轨迹精细度, 指在判断碰撞的时候连续判断多少个点
        ego_vvehicle = leaf._virtual_vehicle_dict.get(self._ego_id)
        # 超速
        speed = ego_vvehicle._scalar_velocity
        speed_threshold = 26.5
        overspeed_penalty_coeff = 3
        if speed > 26.5:
            overspeed_penalty = (
                overspeed_penalty_coeff * 10 * ((speed - speed_threshold) * 6 / 41) ** 2
            )

        # 变道
        if ego_vvehicle._control_action in ["SLIDE_LEFT", "SLIDE_RIGHT"]:
            lanechange_penalty = 2.6 * 2

        # 碰撞
        for vid in leaf._virtual_vehicle_dict.keys():
            if collision_penalty >= 1000:
                break
            # 对每辆车的中间轨迹点判断是否碰撞（避免过程中碰撞的情况检测不到）
            if vid != self._ego_id:
                assis_vego = copy.copy(root._virtual_vehicle_dict.get(self._ego_id))
                assis_vother = copy.copy(root._virtual_vehicle_dict.get(vid))
                for _ in range(traj_accuracy):
                    assis_vego._transform.location += (
                        leaf._virtual_vehicle_dict.get(self._ego_id)._transform.location
                        - root._virtual_vehicle_dict.get(
                            self._ego_id
                        )._transform.location
                    ) / traj_accuracy
                    assis_vother._transform.location += (
                        leaf._virtual_vehicle_dict.get(vid)._transform.location
                        - root._virtual_vehicle_dict.get(vid)._transform.location
                    ) / traj_accuracy
                    if assis_vego.judge_collision(assis_vother):
                        collision_penalty += 1000
                        break
                del assis_vego
                del assis_vother

        return overspeed_penalty + collision_penalty

    @staticmethod
    def ttc_to_risk(ttc):
        """
        将time-to-collision转换成risk
        """
        threshold = 9
        if ttc < 0 or ttc > threshold:
            return 0
        return (threshold - ttc) ** 2
