import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/shared_home/Documents/robo202/tricycle_camera_ws_withoutlidar_withtricycleplugin/install/tricycle_camera_pkg'
