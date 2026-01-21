// In the constructor or reset function
double iDist = 0.1;
double sDist = 0.05;
double damping = 0.0;
addCollisions("ur5e", "kinova_default", {{"*", "*", iDist, sDist, damping}});