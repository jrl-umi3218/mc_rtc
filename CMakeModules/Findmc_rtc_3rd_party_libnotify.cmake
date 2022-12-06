#
# Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
#

include(FindPkgConfig)
pkg_check_modules(LIBNOTIFY REQUIRED libnotify IMPORTED_TARGET)
add_library(mc_rtc_3rd_party::libnotify ALIAS PkgConfig::LIBNOTIFY)
