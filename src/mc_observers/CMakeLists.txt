macro(add_observer observer_base)
  set(observer_SRC ${CMAKE_CURRENT_SOURCE_DIR}/${observer_base}.cpp)
  set(observer_HDR ${MC_RTC_SOURCE_DIR}/include/mc_observers/api.h ${MC_RTC_SOURCE_DIR}/include/mc_observers/${observer_base}.h)
  if(MC_RTC_BUILD_STATIC)
    target_sources(mc_observers PRIVATE ${observer_SRC} ${observer_HDR})
    target_include_directories(mc_observers PRIVATE $<BUILD_INTERFACE:${MC_RTC_SOURCE_DIR}/include/mc_observers>)
  else()
    add_library(${observer_base} SHARED ${observer_SRC} ${observer_HDR})
    set_target_properties(${observer_base} PROPERTIES FOLDER observers)
    set_target_properties(${observer_base} PROPERTIES COMPILE_FLAGS "-DMC_OBSERVER_EXPORTS" PREFIX "")
    target_link_libraries(${observer_base} PUBLIC mc_rbdyn mc_rtc_gui mc_control mc_filter)
    target_include_directories(${observer_base} PUBLIC $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${MC_RTC_SOURCE_DIR}/include/mc_observers>)
    install(TARGETS ${observer_base} EXPORT mc_rtc_observers
      ARCHIVE DESTINATION "${MC_OBSERVERS_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:/debug>"
      LIBRARY DESTINATION "${MC_OBSERVERS_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:/debug>"
      RUNTIME DESTINATION "${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX}$<$<CONFIG:debug>:/debug>"
    )
  endif()
endmacro()

add_observer(EncoderObserver)
add_observer(BodySensorObserver)
add_observer(KinematicInertialPoseObserver)
add_observer(KinematicInertialObserver)
if(NOT MC_RTC_BUILD_STATIC)
  target_link_libraries(KinematicInertialObserver PUBLIC KinematicInertialPoseObserver)
  set_target_properties(KinematicInertialObserver PROPERTIES INSTALL_RPATH ${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX})
endif()

if(NOT MC_RTC_BUILD_STATIC)
  install(
    EXPORT mc_rtc_observers
    NAMESPACE mc_observers::
    DESTINATION "${CONFIG_INSTALL_DIR}"
  )
endif()
