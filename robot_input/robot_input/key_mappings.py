"""
存放所有按键映射配置
"""
BACKTOSTART = 0
DISABLE = 1
JOINTCONTROL = 2
CARTESIAN = 3
MOVEJ = 4
MOVEL = 5
MOVEC = 6
TEACH = 7
TEACHREPEAT = 8
SAVESTATE = 9
LOADSTATE = 10
BACKTOINITIAL = 11
ESC = 12

JOINT1_POSITIVE = 41
JOINT1_NEGATIVE = 42
JOINT2_POSITIVE = 43
JOINT2_NEGATIVE = 44
JOINT3_POSITIVE = 45
JOINT3_NEGATIVE = 46
JOINT4_POSITIVE = 47
JOINT4_NEGATIVE = 48
JOINT5_POSITIVE = 49
JOINT5_NEGATIVE = 50
JOINT6_POSITIVE = 51
JOINT6_NEGATIVE = 52
GRIPPER_OPEN = 53
GRIPPER_CLOSE = 54

FORWARD = 55
BACKWARD = 56
LEFT = 57
RIGHT = 58
UP = 59
DOWN = 60
ROLL_POSITIVE = 61
ROLL_NEGATIVE = 62
PITCH_POSITIVE = 63
PITCH_NEGATIVE = 64
YAW_POSITIVE = 65
YAW_NEGATIVE = 66

# 模式切换按键映射
MODE_KEYS = {
    '`': BACKTOSTART,  # BACKTOSTART
    '1': DISABLE,  # DISABLE
    '2': JOINTCONTROL,  # JOINTCONTROL
    '3': CARTESIAN,  # CARTESIAN
    '4': MOVEJ,  # MOVEJ
    '5': MOVEL,  # MOVEL
    '6': MOVEC,  # MOVEC
    '7': TEACH,  # TEACH
    '8': TEACHREPEAT,  # TEACHREPEAT
    '9': SAVESTATE,  # SAVESTATE
    '0': LOADSTATE, # TOSTATE
    '-': BACKTOINITIAL,  # BACKTOINITIAL
    'esc': ESC
}

# 关节控制按键映射
JOINT_CONTROL_KEYS = {
    # Joint Positive
    'q': JOINT1_POSITIVE, 'w': JOINT2_POSITIVE, 'e': JOINT3_POSITIVE, 'r': JOINT4_POSITIVE, 't': JOINT5_POSITIVE, 'y': JOINT6_POSITIVE,
    # Joint Negative
    'a': JOINT1_NEGATIVE, 's': JOINT2_NEGATIVE, 'd': JOINT3_NEGATIVE, 'f': JOINT4_NEGATIVE, 'g': JOINT5_NEGATIVE, 'h': JOINT6_NEGATIVE,
    # Gripper
    '[': GRIPPER_OPEN, ']': GRIPPER_CLOSE
}

# 笛卡尔控制按键映射
CARTESIAN_CONTROL_KEYS = {
    # Movement
    'q': FORWARD, 'a': BACKWARD,  # X
    'w': LEFT, 's': RIGHT,  # Y
    'e': UP, 'd': DOWN,  # Z
    # Orientation
    'r': ROLL_POSITIVE, 'f': ROLL_NEGATIVE,  # Roll
    't': PITCH_POSITIVE, 'g': PITCH_NEGATIVE,  # Pitch
    'y': YAW_POSITIVE, 'h': YAW_NEGATIVE,  # Yaw
    # Gripper
    '[': GRIPPER_OPEN, ']': GRIPPER_CLOSE
}

# 模式切换前条件(完全准备)
MODE_PREREQUISITES = {
    BACKTOSTART: [i for i in range(12)],
    DISABLE: [i for i in range(12)],
    JOINTCONTROL: [i for i in range(12)],
    CARTESIAN: [i for i in range(12)],
    MOVEJ: [i for i in range(12)],
    MOVEL: [i for i in range(12)],
    MOVEC: [i for i in range(12)],
    TEACH: [i for i in range(12)],
    TEACHREPEAT: [i for i in range(12)],
    SAVESTATE: [i for i in range(12)],
    LOADSTATE: [i for i in range(12)],
    BACKTOINITIAL: [i for i in range(12)],
}

# 模式切换前条件(所有动作执行完后统一切换到JOINTCONTROL模式)

