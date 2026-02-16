import numpy as np
import matplotlib.pyplot as plt
import ezdxf

# --- Establish Constants ---
L1 = 6.700
L2 = 0.000
L3 = 4.431
L4 = 1.100
L5 = 1.169

B1 = 2.677
B2 = 0.349
B3 = 2.446
B4 = 0.304
B5 = 0.507

Rf = 0.6
E = 0.19

# Cam geometry
Rb = 8.8
Rp = Rf + Rb
d = np.sqrt(Rp**2 - E**2)

# Solution region boundaries
b1 = B1
b2 = B1 + B2
b3 = B1 + B2 + B3
b4 = B1 + B2 + B3 + B4
b5 = B1 + B2 + B3 + B4 + B5

# --- Piecewise Motion Laws ---
def pos_1(L1, theta, B1):
    return 6*L1*(theta/B1)**2 - 8*L1*(theta/B1)**3 + 3*L1*(theta/B1)**4

def pos_2(L1):
    return L1

def pos_3(L3, theta, B3):
    return L3*(1 - theta/B3 + (1/np.pi)*np.sin(np.pi*theta/B3)) + (L1 - L3)

def pos_4(theta):
    return -3.622138511*(theta - 5.472) + L1 - L3

def pos_5(L5, theta, B5, b4):
    return L5*(1 - np.sin(np.pi*(theta - b4)/(2*B5)))

def vel_1(L1, theta, B1):
    return 12*L1*theta/B1**2 - 24*L1*theta**2/B1**3 + 12*L1*theta**3/B1**4

def vel_2():
    return 0.0

def vel_3(L3, theta, B3):
    return -(L3/B3)*(1 - np.cos(np.pi*theta/B3))

def vel_4():
    return -3.622138511

def vel_5(L5, theta, B5, b4):
    return -(np.pi*L5)/(2*B5)*np.cos(np.pi*(theta - b4)/(2*B5))

def acc_1(L1, theta, B1):
    return 12*L1/B1**2 - 48*L1*theta/B1**3 + 36*L1*theta**2/B1**4

def acc_2():
    return 0.0

def acc_3(L3, theta, B3):
    return -(np.pi*L3)/B3**2*np.sin(np.pi*theta/B3)

def acc_4():
    return 0.0

def acc_5(L5, theta, B5, b4):
    return (np.pi**2 * L5)/(4*B5**2)*np.sin(np.pi*(theta - b4)/(2*B5))

def pressure_angle(y, yy, E, Rp):
    phi = np.arctan((yy - E)/(y + np.sqrt(Rp**2 - E**2)))
    return np.rad2deg(phi)

def radius_curve(y, yy, yyy, Rp):
    num = (((Rp + y)**2 + yy**2)**1.5)
    den = ((Rp + y)**2 + 2*(yy**2) - yyy*(Rp + y))
    return num / den

def pitch_curve(theta, E, d, y):
    lam = (2*np.pi - theta) - np.arctan(E/(d + y))
    mag = np.sqrt((d + y)**2 + E**2)
    return np.cos(lam)*mag, np.sin(lam)*mag

# --- MAIN ---
def main():
    theta_range = np.linspace(0.0, 360.0, 2000, endpoint=False)

    pos_array = []
    vel_array = []
    acc_array = []
    phi_array = []
    R_array = []
    x_array = []
    y_array = []

    for theta_deg in theta_range:
        theta_rad = np.deg2rad(theta_deg)

        if theta_rad <= b1:
            y = pos_1(L1, theta_rad, B1)
            yy = vel_1(L1, theta_rad, B1)
            yyy = acc_1(L1, theta_rad, B1)

        elif b1 < theta_rad <= b2:
            y = pos_2(L1)
            yy = vel_2()
            yyy = acc_2()

        elif b2 < theta_rad <= b3:
            t = theta_rad - b2
            y = pos_3(L3, t, B3)
            yy = vel_3(L3, t, B3)
            yyy = acc_3(L3, t, B3)

        elif b3 < theta_rad <= b4:
            y = pos_4(theta_rad)
            yy = vel_4()
            yyy = acc_4()

        else:
            y = pos_5(L5, theta_rad, B5, b4)
            yy = vel_5(L5, theta_rad, B5, b4)
            yyy = acc_5(L5, theta_rad, B5, b4)

        pos_array.append(y)
        vel_array.append(yy)
        acc_array.append(yyy)
        phi_array.append(pressure_angle(y, yy, E, Rp))
        R_array.append(radius_curve(y, yy, yyy, Rp))
        x_soln, y_soln = pitch_curve(theta_rad, E, d, y)
        x_array.append(x_soln)
        y_array.append(y_soln)

    # Plots
    plt.plot(theta_range, pos_array, label="y")
    plt.plot(theta_range, vel_array, label="y'")
    plt.plot(theta_range, acc_array, label="y''")
    plt.legend(); plt.grid(True); plt.show()

    plt.plot(theta_range, phi_array, label="φ")
    plt.legend(); plt.grid(True); plt.show()

    plt.plot(theta_range, R_array, label="ρ")
    plt.legend(); plt.grid(True); plt.show()

    plt.plot(x_array, y_array, label="Pitch Curve")
    plt.axis('equal'); plt.legend(); plt.grid(True); plt.show()

    # Export Spline as DXF
    pts = list(zip(np.array(x_array), np.array(y_array)))
    pts.append(pts[0])

    doc = ezdxf.new()
    msp = doc.modelspace()
    msp.add_spline(fit_points=pts, dxfattribs={"flags": 1})
    doc.saveas("cam_pitch_spline.dxf")
    print("Saved cam pitch spline as cam_pitch_spline.dxf")

if __name__ == "__main__":
    x_array, y_array = main()
