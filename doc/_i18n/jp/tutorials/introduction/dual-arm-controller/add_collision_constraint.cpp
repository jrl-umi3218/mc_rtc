// コンストラクタ内、またはreset関数内
double iDist = 0.1;
double sDist = 0.05;
double damping = 0.0;
addDistanceLimits("ur5e", "kinova", {{"*", "*", iDist, sDist, damping}});
