# In the constructor or reset callback
iDist, sDist, damping = 0.1, 0.05, 0.0
self.addCollisions(
    "ur5e",
    "kinova_default",
    [mc_rbdyn.Collision("*", "*", iDist, sDist, damping)],
)
