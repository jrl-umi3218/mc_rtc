# bin/py/test
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from mc_rtc import mc_rbdyn


def test_RobotModule():
    rm = mc_rbdyn.RobotLoader.get_robot_module("JVRC1")
    assert rm.name == "jvrc1"
    rm.name = "toto"
    assert rm.name == "toto"
    assert rm.default_attitude[0] == 1
    assert rm.default_attitude[1] == 0
    assert rm.default_attitude[2] == 0
    assert rm.default_attitude[3] == 0
    assert rm.default_attitude[4] == 0
    assert rm.default_attitude[5] == 0
    assert rm.default_attitude[6] == 0.8275
    assert rm.canonicalParameters[0] == "JVRC1"
    assert len(rm.stance) == 59
    assert len(rm.ref_joint_order) == 44
    # FIXME: devices
    # devices = rm.devices()
    # print(devices[0].name())
    assert len(rm.frames) == 0
    print(rm.frames)


def test_Robot():
    rm = mc_rbdyn.RobotLoader.get_robot_module("JVRC1")
    robots = mc_rbdyn.loadRobot(rm)
    robot = robots.robot()
    assert robot.name() == "jvrc1"
    assert robot.refJointOrder() == rm.ref_joint_order
    assert len(robot.frames()) > 0
    # mbc = robot.mbc()
    # print(mbc.q)
