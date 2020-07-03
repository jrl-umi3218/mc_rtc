# - Dowloads log file from LOG_URL
# - Run the corresponding target EXECUTABLE_TARGET that was used to generate this log file (with optional parameters ARGN).
#   This will generates a new log file whose path should be specified with GENERATED_LOG
# - Uses mc_bin_compare tool to compare the entries in both logs
macro(mc_rtc_log_test NAME LOG_URL EXECUTABLE_TARGET GENERATED_LOG)
  set(REFERENCE_LOG ${CMAKE_CURRENT_BINARY_DIR}/reference_log.bin)
  message(STATUS "Generating ${CMAKE_CURRENT_BINARY_DIR}/test_log_${NAME}.sh")

file(GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_log_${NAME}.sh"
  CONTENT
"
#!/bin/sh
wget -O ${REFERENCE_LOG} ${LOG_URL}
$<TARGET_FILE:mc_sample_CoMGeneration> ${ARGN}
$<TARGET_FILE:mc_bin_compare> ${REFERENCE_LOG} ${GENERATED_LOG}
")

### file(GENERATE...) does not support permissions, and the files are not generated until GENERATE time (e.g after all the cmake scripts have been processed). Thus the only way to set permissions would be at build time, which involves creating extra custom targets

# Compare logs (will probably only work on linux)
  add_test(NAME TestRegressionCoMGeneration
    COMMAND "/bin/sh ${CMAKE_CURRENT_BINARY_DIR}/test_log_${NAME}.sh")
endmacro()

if(TARGET mc_sample_CoMGeneration)
  # Test regression of the CoM trajectory generation using the mc_sample_CoMGeneration sample
  mc_rtc_log_test(
    TestRegressionCoMGeneration
    "https://seafile.lirmm.fr/f/75870c0ca45f469591c1/?dl=1"
    mc_sample_CoMGeneration
    "/tmp/mc_rtc-test-CoMGenerator-latest.bin"
  )
endif()
