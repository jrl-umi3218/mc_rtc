import mc_rtc
import numpy as np

c = mc_rtc.Configuration()
c.add_null("null_key")
c.array("array_reserve", 20)
arr = c.array("array")
arr.push(42)
arr.push(42.)

vec2 = np.array([1, 2])
arr.push(vec2)
