include(../../../cmake/python.cmake)

findpython()

add_controller("python" "python_controller.cpp" "python_controller.h")
target_include_directories(python PRIVATE ${PYTHON_INCLUDE_DIRS} ../../../binding/python)
target_link_libraries(python PUBLIC ${PYTHON_LIBRARIES})

add_dependencies(python mc_rtc-python-bindings)
