import numpy as np
import mc_rtc

c = mc_rtc.Configuration()
c.add_null("null_key")

arr = c.array("empty_array")
arr.push(3.5)
print(c["empty_array"])
v = 3.0
