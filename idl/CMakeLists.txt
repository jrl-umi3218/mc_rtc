#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

find_program(FLATC flatc)
if(FLATC)
  set(FBS_IN ${CMAKE_SOURCE_DIR}/idl/MCLog.fbs)
  set(HEADERS_OUT_DIR ${CMAKE_SOURCE_DIR}/include/mc_rtc/log/serialization)
  set(HEADERS_OUT ${HEADERS_OUT_DIR}/MCLog_generated.h)
  add_custom_command(
    OUTPUT ${HEADERS_OUT}
    COMMAND flatc
    ARGS --cpp -o ${HEADERS_OUT_DIR} ${FBS_IN}
    DEPENDS ${FBS_IN}
    COMMENT "Generating flatbuffer header for ${FBS_IN}"
    )
  add_custom_target(generate_mc_log_code ALL DEPENDS ${HEADERS_OUT})
endif()
