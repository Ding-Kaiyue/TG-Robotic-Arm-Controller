"""
存放所有界面提示消息
"""

# 主菜单消息
MAIN_MENU = """
Reading from the keyboard and Publishing to Keyboard_control_mode!
----------------------------------------------------------------
`: BACKTOSTART
1: DISABLE
2: JOINTCONTROL
3: CARTESIAN
4: MOVEJ
5: MOVEL
6: MOVEC
7: TEACH
8: TEACHREPEAT
9: SAVESTATE
0: LOADSTATE
-: BACKTOINITIAL

CTRL-C to quit.
"""
# 返回初始位置
BACKTOSTART_MSG = """
Back To Start Mode - All joints are going to be start moveing positions.
"""

# 静止模式消息
DISABLE_MSG = """
DISABLE Mode - All joints are disable.
"""

# 关节控制模式消息
JOINTCONTROL_MSG = """
Joint Speed Control Mode - Control each joint individually
-------------------------------------------------------
| Joint |  1  |  2  |  3  |  4  |  5  |  6  | Gripper |
|-------|-----|-----|-----|-----|-----|-----|---------|
|  Pos  |  q  |  w  |  e  |  r  |  t  |  y  |    [    |
|  Neg  |  a  |  s  |  d  |  f  |  g  |  h  |    ]    |
-------------------------------------------------------
Press ESC to exit mode
"""

# 笛卡尔控制模式消息
CARTESIAN_MSG = """
Cartesian Control Mode - Control end effector pose
----------------------------------------------------------------
| Action      | Pos Key | Neg Key |
|-------------|---------|---------|
| X Movement  |    q    |    a    |
| Y Movement  |    w    |    s    |
| Z Movement  |    e    |    d    |
| Roll        |    r    |    f    |
| Pitch       |    t    |    g    |
| Yaw         |    y    |    h    |
| Gripper     |    [    |    ]    |
----------------------------------------------------------------
Press ESC to exit
"""

# MOVEJ模式消息
MOVEJ_MSG = """
MoveJ Mode - Move to a joint space target position
Input six joint angles separated by spaces and press Enter twice to execute.
For example: 0.5 0.1 0.1 0.5 -0.2 0.5
(Press Enter once to continue inputting, and press Enter twice to start the movement.)
----------------------------------------------------------------
Press ESC to exit mode
"""

# MOVEL模式消息
MOVEL_MSG = """
MoveL Mode - Move to a Cartesian space target position
Input the target pose: XYZ + Euler angles (six values in total), and press Enter twice to execute.
For example: 0.5 0.1 0.1 0.5 -0.2 0.5
(Press Enter once to continue inputting, and press Enter twice to start the movement.)
----------------------------------------------------------------
Press ESC to exit mode
"""

# MOVEC模式消息
MOVEC_MSG = """
MOVEC Mode - Move to a target position using a circular path
First, input the intermediate point (six values) and press Enter.
Then, input the end - point (six values) and press Enter twice to execute.
Example:
Intermediate point: 0 0 0 0.45 0 0.4
End - point: 0 0 0 0.45 0.2 0.2
----------------------------------------------------------------
Press ESC to exit mode
"""

# TEACH模式消息
TEACH_MSG = """
Teach Mode - Enter the teaching state to record robotic arm trajectories
First, input the trajectory label name and press Enter.
After that, you can drag the robotic arm to move along a certain trajectory.
The robotic arm will continuously record the trajectory until you press Enter .
The recorded trajectory will be saved in a your_label_name.db3 file 
in the your_workspace_folder/config directory.
----------------------------------------------------------------
Press ESC to exit mode
"""

# TEACHREPEAT模式消息
TEACHREPEAT_MSG = """
Teachrepeat Mode - Repeat the recorded trajectory
First, input the trajectory label name and press Enter.
After that, the robotic arm will repeat this recorded trajectory.
----------------------------------------------------------------
Press ESC to exit mode
"""

# SAVESTATE模式消息
SAVESTATE_MSG = """
Save State Mode - Save the current robotic arm state
When you enter this mode, please input your prefer state_label_name, 
the current robotic arm state will be saved in 
your_workspace_folder/config/arm_states_saved.db3.
After that, the robotic arm state will be JOINTCONTROL.
----------------------------------------------------------------
Press ESC to exit mode
"""

# LOADSTATE模式消息
LOADSTATE_MSG = """
LOAD STATE Mode - Move the robotic arm to a saved state
When you enter this mode, please input the state_label_name,
the robotic arm will move to the saved state by MOVEJ.
After that, the robotic arm state will be JOINTCONTROL.
"""

# BACKTOINITIAL_MSG模式消息
BACKTOINITIAL_MSG = """
Back To Initial Mode - All joints are going to be initial positions.
"""