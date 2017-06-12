include(../../../cmake/python.cmake)

include_directories(../../../binding/python)

FINDPYTHON()
include_directories(${PYTHON_INCLUDE_DIRS})

add_controller("python" "python_controller.h" "python_controller.cpp")
target_link_libraries(python ${PYTHON_LIBRARIES})

ADD_DEPENDENCIES(python mc_rtc-python-bindings)
