class Decision:
    """
    决策基类
    """

    def __init__(self) -> None:
        self._decision = "MAINTAIN"

    def decide(self):
        """
        返回控制结果
        """
        return self._decision


class LaplaceDecision(Decision):
    """
    CDM基于叶子结点的决策: 等可能决策法
    """

    def __init__(self) -> None:
        super().__init__()

    def get_decision(self, reward_dict, lon_num, lat_num, ego_reward_dict):
        """获得决策结果"""
        for key in reward_dict.keys():
            if key in ["ACCELERATE", "DECELERATE", "MAINTAIN"]:
                reward_dict[key] = sum(reward_dict[key]) / (lon_num if lon_num != 0 else 1)
            else:
                reward_dict[key] = sum(reward_dict[key]) / (3 * lat_num if lat_num != 0 else 1)
        self._decision = max(reward_dict, key=reward_dict.get)
        print("decision is:", self._decision)
        return self._decision
    
    def get_ego_decision(self, ego_reward_dict):
        max_reward = ego_reward_dict["MAINTAIN"]
        max_reward_action = "MAINTAIN"
        for key in ego_reward_dict.keys():
            if max_reward < ego_reward_dict[key]:
                max_reward = ego_reward_dict[key]
                max_reward_action = key 
        print("decision is:", max_reward_action)
        return max_reward_action
