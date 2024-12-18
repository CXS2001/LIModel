from Cognitive_Driver_Model.dstructures.node import CIPORoot
from Cognitive_Driver_Model.dstructures.node import EgoNode
from Cognitive_Driver_Model.envs.world import CarModule
# from dstructures.node import CIPORoot
# from dstructures.node import EgoNode
# from envs.world import CarModule


class EnumerateTree(CarModule):
    """
    包含根节点和叶子节点
    """

    def __init__(self, ego_id, main_id, map) -> None:
        super().__init__(ego_id, main_id, map)
        self._root = None
        self._leaves = []
        self._num_lon_leaves = 0
        self._num_lat_leaves = 0

        self._ego_root = None
        self._ego_leaves = []

    def generate_root_from_cipo(
        self, real_vehicle_dict, close_vehicle_id_list, lon_levels, lat_levels
    ):
        """
        从CIPO Observer生成Root节点
        """
        virtual_vehicle_dict = {}
        for cid in close_vehicle_id_list:
            virtual_vehicle_dict[cid] = real_vehicle_dict.get(cid).clone_to_virtual()
        self._root = CIPORoot(
            self._ego_id,
            self._main_id,
            self._map,
            virtual_vehicle_dict,
            lon_levels,
            lat_levels,
        )

    def grow_tree(self):
        """
        由根节点生成第一层叶子结点
        返回第一层叶子节点列表与两类叶子节点的数量
        """
        if self._root == None:
            raise ValueError("You have not generate a root node.")

        lon_leaves, num_lon = self._root.generate_leaves("longitude")
        lat_leaves, num_lat = self._root.generate_leaves("lateral")
        self._leaves = lon_leaves + lat_leaves
        self._num_lon_leaves = num_lon
        self._num_lat_leaves = num_lat

        return self._leaves, num_lon, num_lat



    def generate_ego_root(self, real_vehicle_dict):
        virtual_ego_vehicle = real_vehicle_dict[self._ego_id].clone_to_virtual()
        self._ego_root = EgoNode(
            self._ego_id,
            self._map,
            virtual_ego_vehicle,
        )


    def grow_ego_tree(self):
        self._ego_tree = {}
        for depth in range(1, 5):
            self._ego_tree[depth] = []
            if depth == 1:
                if self._ego_root == None:
                    raise ValueError("You have not generate a root node.")
                else:
                    self._ego_tree[depth] = self._ego_root.generate_ego_leaves()
                    for i in range(len(self._ego_tree[depth])):
                        self._ego_tree[depth][i]._father = 0

            else:
                for i in range(len(self._ego_tree[depth - 1])):
                    temp_node = self._ego_tree[depth - 1][i].generate_ego_leaves()
                    for j in range(len(temp_node)):
                        temp_node[j]._father = i
                    self._ego_tree[depth].extend(temp_node)
                
        return self._ego_tree