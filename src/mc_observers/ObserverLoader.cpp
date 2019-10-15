/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_observers/ObserverLoader.h>

std::unique_ptr<mc_rtc::ObjectLoader<mc_observers::Observer>> mc_observers::ObserverLoader::observer_loader;
bool mc_observers::ObserverLoader::enable_sandbox_ = false;
bool mc_observers::ObserverLoader::verbose_ = false;
std::mutex mc_observers::ObserverLoader::mtx{};
