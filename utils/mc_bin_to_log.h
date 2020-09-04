/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <string>
#include <vector>

/**
 * @brief Convert from bin to csv
 *
 * @param in Input path to the bin log
 * @param out Output path to the csv file
 * @param entriesFilter Name of entries to convert. When empty, convert all
 * entries
 */
void mc_bin_to_log(const std::string & in, const std::string & out, const std::vector<std::string> & entiesFilter = {});
