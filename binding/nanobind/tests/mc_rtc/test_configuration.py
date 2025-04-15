import numpy as np
import mc_rtc

c = mc_rtc.Configuration()
c.add_null("null_key")

arr = c.array("empty_array")
arr.push(3.5)

np_vec2 = np.array([1.0, 2.0])
c.add("vec2", np_vec2)
c.add("float", 3.0)

c2 = mc_rtc.Configuration()
c.add("config", c2)
