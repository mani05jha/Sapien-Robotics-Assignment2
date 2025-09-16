import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/maninderjha/Videos/sapien_robotics/install/sapien_robotics'
