macro(add_observer observer_base)
  set(observer_SRC ${observer_base}.cpp)
  set(observer_HDR ../../include/mc_observers/api.h ${observer_base}.h)
  add_library(${observer_base} SHARED ${observer_SRC} ${observer_HDR})
  set_target_properties(${observer_base} PROPERTIES COMPILE_FLAGS "-DMC_OBSERVERS_EXPORTS" PREFIX "")
  target_link_libraries(${observer_base} PUBLIC mc_rbdyn mc_rtc_gui)
  target_include_directories(${observer_base} PRIVATE "../../include/mc_observers")
  install(TARGETS ${observer_base} DESTINATION ${MC_OBSERVERS_INSTALL_PREFIX})
endmacro()

add_observer(FloatingBasePosObserver)
add_observer(FloatingBasePosVelObserver)
### FIXME @gergondet
## Should be the correct way, but does not work
# target_link_libraries(FloatingBasePosVelObserver PUBLIC FloatingBasePosObserver)
## But this hack works fine
target_link_libraries(FloatingBasePosVelObserver PUBLIC ${MC_OBSERVERS_INSTALL_PREFIX}/FloatingBasePosObserver.so)
set_target_properties(FloatingBasePosVelObserver PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
add_observer(BodySensorObserver)
# add_observer(EncoderVelocityObserver)
