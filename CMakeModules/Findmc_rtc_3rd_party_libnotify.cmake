#
# Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
#

if(NOT COMMAND pkg_check_modules)
  find_package(PkgConfig)
endif()
pkg_check_modules(mc_rtc_3rd_party_libnotify REQUIRED libnotify IMPORTED_TARGET)
