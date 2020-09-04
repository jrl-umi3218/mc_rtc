/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <string>
#include <vector>

/**
 * @brief Convert from bin to flat
 *
 * @param in Input path to the bin log
 * @param out Output path to the flatlog
 * @param entriesFilter Name of entries to convert. When empty, convert all
 * entries
 */
void mc_bin_to_flat(const std::string & in,
                    const std::string & out,
                    const std::vector<std::string> & entriesFilter = {});
