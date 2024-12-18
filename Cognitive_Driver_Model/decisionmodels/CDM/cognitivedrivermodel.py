"""
Cognitive Driver Model
""" 
class CognitiveDriverModel:
    def __init__(
        self,
        driving_style,
        destination,
        observer,
        enumeratetree,
        risk_calculator,
        reward_calculator,
        decision_mode,
    ) -> None:
        self._driving_style = driving_style
        self._destination = destination
        self._observer = observer
        self._enumeratetree = enumeratetree
        self._risk_calculator = risk_calculator
        self._reward_calculator = reward_calculator
        self._decision_mode = decision_mode


class NormalCognitiveDriverModel(CognitiveDriverModel):
    def __init__(
        self,
        driving_style,
        destination,
        observer,
        enumeratetree,
        risk_calculator,
        reward_calculator,
        decision_mode,
    ) -> None:
        super().__init__(
            driving_style,
            destination,
            observer,
            enumeratetree,
            risk_calculator,
            reward_calculator,
            decision_mode,
        )

    def run_forward(self, full_vehicle_id_dict, nnet=None):
        """前向计算一轮"""
        # long-term decision
        if self._observer.if_dest_in_front(full_vehicle_id_dict, self._destination):

            self._enumeratetree.generate_ego_root(full_vehicle_id_dict)
            ego_tree = self._enumeratetree.grow_ego_tree()

            risk_dict = self._risk_calculator.cal_ego_tree_risk(ego_tree, self._destination)

            ego_reward_dict = self._reward_calculator.cal_ego_reward_dict(risk_dict, self._driving_style)

        close_vehicle_id_list, lon_levels, lat_levels = self._observer.observe(
            full_vehicle_id_dict
        )

        self._enumeratetree.generate_root_from_cipo(
            full_vehicle_id_dict, close_vehicle_id_list, lon_levels, lat_levels
        )
        leaves,  num_lon, num_lat = self._enumeratetree.grow_tree()

        self._risk_calculator.cal_risk_list(self._enumeratetree._root, leaves)

        preferred_leaves, other_leaves = self._risk_calculator.get_preferred_leaves(
            leaves, self._driving_style
        )

        reward_dict = self._reward_calculator.cal_reward_dict(
            preferred_leaves, other_leaves
        )

        return self._decision_mode.get_decision(reward_dict, num_lon, num_lat, ego_reward_dict)

   




class AdversarialCognitiveDriverModel(CognitiveDriverModel):
    def __init__(
        self,
        driving_style,
        destination,
        observer,
        enumeratetree,
        risk_calculator,
        reward_calculator,
        decision_mode,
    ) -> None:
        super().__init__(
            driving_style,
            destination,
            observer,
            enumeratetree,
            risk_calculator,
            reward_calculator,
            decision_mode,
        )

    def run_forward(self, full_vehicle_id_dict, nnet):
        """前向计算一轮"""
        # 观测器返回结果, ACDM采用CIPO观测器, 返回三个结果
        close_vehicle_id_list, lon_levels, lat_levels = self._observer.observe(
            full_vehicle_id_dict
        )

        # 树生成根节点
        self._enumeratetree.generate_root_from_cipo(
            full_vehicle_id_dict, close_vehicle_id_list, lon_levels, lat_levels
        )

        # 树生成叶子结点
        leaves, num_lon, num_lat = self._enumeratetree.grow_tree()

        # 计算风险
        self._risk_calculator.cal_risk_list(self._enumeratetree._root, leaves)

        # 筛选叶子结点
        preferred_leaves, other_leaves = self._risk_calculator.get_preferred_leaves(
            leaves, self._driving_style
        )

        # 计算收益
        reward_dict = self._reward_calculator.cal_reward_dict(
            self._enumeratetree._root, preferred_leaves, other_leaves, nnet
        )

        # 决策
        return self._decision_mode.get_decision(reward_dict, num_lon, num_lat)