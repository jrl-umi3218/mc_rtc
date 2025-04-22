#
# In order for stubs to work correctly, we need to import each symbol as
# from .mc_rtc_python import Symbol as Symbol
#
# The "as Symbol" is used to bring the otherwise private symols into the public namespace

# from .mc_rtc_python import Configuration as Configuration
# from .mc_rtc_python import mc_rbdyn as mc_rbdyn
from .mc_rtc_python import *

# FIXME: how to import this for stubs
# from .mc_rtc_python.mc_rbdyn import RobotModule as RobotModule
__all__ = ["Configuration", "mc_rbdyn"]
