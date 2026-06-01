# deadwheel-code-samples

EQUATIONS NEEDED:/
Odometry: /
C = π × diameter /
ΔL = L_current − L_previous /
ΔP = P_current − P_previous /
Δfwd = (ΔL / T) × C /
Δstr = (ΔP / T) × C /
θ = IMU reading /
Δx = (Δfwd × cosθ) − (Δstr × sinθ) /
Δy = (Δfwd × sinθ) + (Δstr × cosθ) /
x = x + Δx /
y = y + Δy /
PD controller: /
error = target − current /
pTerm = error × kP /
deltaError = error − lastError /
dTerm = deltaError × kD /
power = pTerm + dTerm /
lastError = error /
