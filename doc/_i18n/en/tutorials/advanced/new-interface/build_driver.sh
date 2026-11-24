cd my_robot_driver
cmake -S . -B build \
  -DCMAKE_PREFIX_PATH=/path/to/install \
  -DCMAKE_INSTALL_PREFIX=/path/to/install
cmake --build build
cmake --install build
