import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rama/Documents/code/Docker/moonshot_project/ros2_ws/install/moving_table_pkg'
