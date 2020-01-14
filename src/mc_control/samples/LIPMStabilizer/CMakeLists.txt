add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/LIPMStabilizer.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMStabilizer.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMStabilizer.yaml" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")

