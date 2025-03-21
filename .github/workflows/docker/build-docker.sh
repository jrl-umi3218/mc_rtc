#!/bin/bash


ls -lr /source &&
cd /source/mc-rtc-superbuild &&
./utils/bootstrap-linux.sh &&
git config --global user.email "arn.tanguy@gmail.com" &&
git config --global user.name "Arnaud Tanguy (Automated CI update)" &&
mkdir -p ${TMPDIR-/tmp}/build-mc-rtc &&
cmake -S /source/mc-rtc-superbuild -B ${TMPDIR-/tmp}/build-mc-rtc -DCMAKE_BUILD_TYPE=RelWithDebInfo -DVERBOSE_TEST_OUTPUT=ON &&
cmake --build ${TMPDIR-/tmp}/build-mc-rtc --config RelWithDebInfo
