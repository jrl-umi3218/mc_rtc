#!/usr/bin/env python2

import os
import sys

if __name__ == "__main__":
    assert(len(sys.argv) == 3)
    ctl = sys.argv[1]
    nrIter = int(sys.argv[2])
    assert(nrIter > 0)
    log_path = "/tmp/mc-rtc-test-{0}-latest.log".format(ctl)
    assert(os.path.exists(log_path))
    with open(log_path) as fd:
        lines = fd.readlines()
        assert(len(lines) == nrIter + 1)
        t = [ int(round(float(lines[i].split(';')[0])/0.005)) for i in xrange(1, nrIter + 1) ]
        assert(t == range(nrIter))
    sys.exit(0)
