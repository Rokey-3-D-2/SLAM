import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/choin/rokey_ws/src/rokey_pjt/install/rokey_pjt'
