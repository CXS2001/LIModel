# 车辆信息(tesla model3)
CAR_LENGTH = 4.791779518127441
CAR_WIDTH = 2.163450002670288
CAR_HEIGHT = 1.4876600503921509

# CDM决策频率
DECISION_DT = 1

# 场景更新频率
STEP_DT = 0.02

# 变道所需时间/s
LANE_CHANGE_TIME = 1

# 场景信息
NUM_LANE = 3
LANE_ID = {"Left": 2, "Middle": 3, "Right": 4}

# 观测距离
OBSERVE_DISTANCE = 30
LONG_TERM_OBSERVE_DISTANCE = 300

# 最大风险值
MAXRISK = 999

# Road ID and Length
ROADS_LENGTH = {
    0: [0.00, 0.00],
    37: [814.84, 0.00],
    2344: [28.99, 814.84],
    38: [300.24, 843.83],
    12: [24.93, 1144.07],
    34: [276.20, 1169.00],
    35: [21.07, 1445.20],
    2035: [29.01, 1466.27],
    36: [12.21, 1495.28],
}  # [该road的长度, 之前road的总长度]

# 一圈总长度
ROUND_LENGTH = 1507.49

# 横纵动作空间
ACTION_SPACE = ["MAINTAIN", "ACCELERATE", "DECELERATE", "SLIDE_LEFT", "SLIDE_RIGHT"]
LON_ACTIONS = ["MAINTAIN", "ACCELERATE", "DECELERATE"]
LAT_ACTIONS = ["SLIDE_LEFT", "SLIDE_RIGHT"]

# 不同动作对应的纵加速度
LON_ACC_DICT = {
    "MAINTAIN": 0,
    "ACCELERATE": 1,
    "DECELERATE": -3,
    "SLIDE_LEFT": 0,
    "SLIDE_RIGHT": 0,
}

# 动作空间对应的reward分数
ACTION_SCORE = {
    "MAINTAIN": 3,
    "ACCELERATE": 4,
    "DECELERATE": 1,
    "SLIDE_LEFT": 3,
    "SLIDE_RIGHT": 3,
}

# 神经网络参数
INPUT_SIZE = 5
HIDDEN_SIZE = 64
NUM_CLASSES = 4
NUM_LAYERS = 2
