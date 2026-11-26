# In the constructor or reset callback
iDist, sDist, damping = 0.1, 0.05, 0.0
self.addDistanceLimits(
    "ur5e",
    "kinova",
    [mc_rbdyn.DistanceLimit("*", "*", iDist, sDist, damping)],
)
