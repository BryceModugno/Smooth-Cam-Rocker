# Cam Pitch Curve Generator

Analytical cam design study in which a plate cam was synthesized to match a moving conveyor’s velocity during a simulated spot-welding operation. The piecewise motion law was implemented in Python to generate a closed CAD-ready pitch curve (DXF spline export).

---

## Design Scenario

A conceptual assembly-line operation requires a spot weld on a part moving vertically on a conveyor belt. To avoid stopping the conveyor for each weld, the welding tip must match the belt velocity over a short distance during the welding operation. The welding tip is mounted on the follower of a plate cam driven at a constant 58 rpm. The conveyor moves at a constant velocity of −22 in/sec (downward).

Follower requirements:

- Start at bottom of lift  
- Rise 6.7 in  
- Dwell at top for 20° of cam rotation  
- During return:
  - Accelerate to −22 in/sec
  - Maintain that velocity for 1.1 in (velocity matching region)
  - Decelerate to bottom  
- Immediately repeat cycle (no bottom dwell)

Because standard trigonometric motion laws force zero acceleration at the start of lift, they were unsuitable for the full-rise section. A custom polynomial motion law was selected to better satisfy the acceleration boundary conditions:

\[
$y = 6L\left(\frac{\theta}{\beta}\right)^2 - 8L\left(\frac{\theta}{\beta}\right)^3 + 3L\left(\frac{\theta}{\beta}\right)^4$
\]

---

## Analytical Development & Verification

Before writing the generator script, the cam motion was developed analytically as a design study. This includes non-zero interface conditions, governing equations for each motion segment, and continuity checks at segment boundaries, followed by numerical verification and plotting.

Included in the derivation package:

- Non-zero interface conditions and continuity verification across segment transitions  
- Governing equations for each motion segment (rise, dwell, return, velocity-matching region)  
- Position / velocity / acceleration profiles and plots  
- Pressure angle and radius of curvature formulations (as applicable)  
- Cross-checks between the analytical expressions and the generated motion curves  

📄 **Analytical derivations + plots (PDF):**  
[View the full derivation package](Media/cam_analytical_derivation.pdf)

---

## Implementation Summary

Engineering workflow used in the Python tool:

1. Define piecewise follower displacement function  
2. Compute velocity and acceleration analytically  
3. Evaluate pressure angle and radius of curvature  
4. Transform follower motion into pitch curve coordinates  
5. Export a closed spline as DXF for CAD integration  

---

## Key Code Snippets

Pitch curve coordinate transform:

```python
def pitch_curve(theta, E, d, y):
    lam = (2*np.pi - theta) - np.arctan(E/(d + y))
    mag = np.sqrt((d + y)**2 + E**2)
    return np.cos(lam)*mag, np.sin(lam)*mag
