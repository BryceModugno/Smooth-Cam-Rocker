# Cam Pitch Curve Generator

Analytical cam design study in which a plate cam was synthesized to match a moving conveyor’s velocity during a simulated spot-welding operation. The resulting motion law was implemented in Python to generate a closed CAD-ready pitch curve (DXF spline export).

---

## Result

<p align="center">
  <img src="Media/cam_pitch_demo.gif" width="600">
</p>

---

## Design Scenario

A conceptual assembly-line operation requires a spot weld on a part moving vertically on a conveyor belt.  
To avoid stopping the conveyor for each weld, the welding tip must match the belt velocity over a short distance during the welding operation.

The welding tip is mounted to the follower of a plate cam driven at a constant 58 rpm.  
The conveyor moves at a constant velocity of −22 in/sec (downward).

Follower requirements:

- Start at bottom of lift  
- Rise 6.7 in  
- Dwell at top for 20° of cam rotation  
- During return:
  - Accelerate to −22 in/sec
  - Maintain that velocity for 1.1 in (velocity matching region)
  - Decelerate to bottom
- Immediately repeat cycle (no bottom dwell)

Because standard trigonometric motion laws force zero acceleration at the start of lift, they were unsuitable for the full-rise section.  
A custom polynomial motion law was selected to better satisfy the acceleration boundary conditions:

\[
y = 6L\left(\frac{\theta}{\beta}\right)^2
- 8L\left(\frac{\theta}{\beta}\right)^3
+ 3L\left(\frac{\theta}{\beta}\right)^4
\]

---

## Design Process

1. Define piecewise follower displacement function  
2. Compute velocity and acceleration analytically  
3. Evaluate pressure angle and radius of curvature  
4. Transform follower motion into pitch curve coordinates  
5. Export closed spline as DXF for CAD integration  

---

## Key Geometry Transformation

```python
def pitch_curve(theta, E, d, y):
    lam = (2*np.pi - theta) - np.arctan(E/(d + y))
    mag = np.sqrt((d + y)**2 + E**2)
    return np.cos(lam)*mag, np.sin(lam)*mag
