cmake_minimum_required(VERSION 3.1)

find_package(mc_rtc REQUIRED)

add_robot_simple(MyRobot)
# 以下のためのショートカット:
# add_robot(MyRobot MyRobot.h MyRobot.cpp)
