import mc_rtc

c = mc_rtc.Configuration()
c.add_null("null_key")
c.array("array_reserve", 20)
arr = c.array("array")
arr.push(42)
arr.push(42.)
