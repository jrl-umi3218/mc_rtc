#
# Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
#

import mc_control
import mc_rtc


class TestPythonState(mc_control.fsm.PythonState):
    def __init__(self):
        self.iter = 0
        self.config = mc_rtc.Configuration()

    def configure(self, config):
        self.config.load(config)

    def start(self, ctl):
        self.value = self.config("value", 0)
        self.output("Value={}".format(self.value))

    def run(self, ctl):
        return True

    def teardown(self, ctl):
        pass
