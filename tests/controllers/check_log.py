#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

import glob
import os
import subprocess
import sys

if __name__ == "__main__":
    assert(len(sys.argv) == 4)
    ctl = sys.argv[1]
    nrIter = int(sys.argv[2])
    utils_dir = sys.argv[3]
    assert(nrIter > 0)
    if os.name == "nt":
        bin_path = max(glob.glob(os.environ['TEMP'] + "/mc-rtc-test-{0}-*.bin".format(ctl)), key = os.path.getctime)
    else:
        sym_path = "/tmp/mc-rtc-test-{0}-latest.bin".format(ctl)
        bin_path = os.path.realpath(sym_path)
        os.remove(sym_path)
    assert(os.path.exists(bin_path))
    log_path = bin_path.replace('.bin', '.log')
    subprocess.call([utils_dir + "/mc_bin_to_log", bin_path, log_path])
    assert(os.path.exists(log_path))
    with open(log_path) as fd:
        lines = fd.readlines()
        print("len(lines) is {}".format(len(lines)))
        assert(len(lines) == nrIter + 1)
        t_col = lines[0].split(';').index('t')
        t = [ int(round(float(lines[i].split(';')[t_col])/0.005)) for i in range(1, nrIter + 1) ]
        assert(t == list(range(nrIter)))
    os.remove(bin_path)
    os.remove(log_path)
    sys.exit(0)
