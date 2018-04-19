include(../../../cmake/python.cmake)

include_directories(../../../binding/python)

findpython()

add_controller("python" "python_controller.cpp" "python_controller.h")
target_include_directories(python PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(python ${PYTHON_LIBRARIES})

add_dependencies(python mc_rtc-python-bindings)
