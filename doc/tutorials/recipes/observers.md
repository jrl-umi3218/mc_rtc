---
layout: tutorials
---

Observers have been introduced with the [merge request 168](https://gite.lirmm.fr/multi-contact/mc_rtc/merge_requests/168).

In order to use the default observer, in your mc_rtc.conf add
```json
"RunObservers": ["Encoder"],
"UpdateObservers": ["Encoder"],
```

In Rviz, create a new robot, give it the same Robot Description path as for your "normal robot" (for example "/control/robot_description"), set TF Prefix to "/real" and choose transparency Alpha = 0.5.
Both robot visualizations should overlap each other in the beginning, but may diverge during runtime.
For example, just let the HRP4 run against an obstacle and you will see the "observed robot" fall, while your "normal robot" stays upright.
