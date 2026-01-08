import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/container_user/wheelchair2/src/install/wheelchair2_description'
