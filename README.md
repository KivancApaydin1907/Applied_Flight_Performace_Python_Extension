# T-38 Talon 3-DOF Flight Simulator (Python & DualSense)

A real-time, 3-Degree-of-Freedom (3-DOF) flight dynamics simulation of the Northrop T-38 Talon. This project bridges rigorous aerodynamic theory with interactive software, featuring a custom physics engine, a cascaded PID autopilot, and hardware-in-the-loop integration using a PlayStation DualSense controller.

## 🚀 Core Features

* **Real-Time Physics Engine:** Custom 6-state numerical integration handling velocity, pitch, pitch rate, and spatial coordinates.
* **DualSense Hardware Integration:** Utilizes `pydualsense` for active haptic feedback, including dynamic transonic and supersonic vibration effects (Mach 1.0+ rumble).
* **Fly-by-Wire Envelope Protection:** Active Alpha (AoA) and G-limiters to prevent structural damage or departures from controlled flight.
* **Cascaded PID Autopilot:** Matrix-scheduled gains based on dynamic pressure, controlling target altitude, pitch limits, and Mach hold.
* **Dual-Display Output:** A PyGame-based interactive HUD for raw telemetry and a PyVista GPU-accelerated 3D out-the-window view.

---

## 📐 Numerical Integration: Why RK4 over Euler?

This simulation utilizes the **4th-Order Runge-Kutta (RK4)** method for advancing the state matrix rather than the simpler Forward Euler method. 

In flight dynamics, the equations of motion are highly non-linear. The Euler method assumes that the rate of change (the derivative) remains constant across the entire time step (`dt`). For an aircraft executing aggressive pitch maneuvers or encountering rapid aerodynamic load shifts, this assumption causes the numerical solution to "drift" rapidly from reality, leading to unbounded errors and mathematical instability.

RK4 solves this by evaluating the derivatives at four distinct points within every single time step:
1.  **k1:** The initial slope at the current state.
2.  **k2:** The slope at the midpoint, using `k1` to predict the state.
3.  **k3:** The slope at the midpoint again, refining the prediction using `k2`.
4.  **k4:** The slope at the end of the step, using `k3`.

By taking a weighted average of these four slopes, RK4 effectively neutralizes the truncation error inherent in Euler's method. This allows the T-38 simulation to maintain extreme accuracy and stability, even during high-G loads or accelerated stalls, without requiring computationally expensive, infinitesimally small time steps.

---

## 🛠️ Prerequisites & Installation

### Dependencies
Ensure you have Python 3.8+ installed. Install the required libraries using pip:

```bash
pip install pygame pyvista numpy pydualsense
```

# Required Files
To run the simulation successfully, ensure the following files are in your working directory:

* main_sim.py (The main execution script)

* flight_dynamics.py & atmosphere.py (Physics modules)

* T_38_General_Data.py (Aircraft aerodynamic derivatives)

* control_systems.py (PID controllers)

* t38_control_matrix.pkl (Pre-calculated matrix for the Autopilot)

* T38.stl (3D mesh for the PyVista visualization)

## 🕹️ Controls & Operation
You can fly the simulation using either a keyboard or a PlayStation DualSense controller.

**Gamepad (DualSense)**
* Right Stick X/Y: Orbit Camera (Azimuth/Elevation)

* Left Stick Y (Axis 1): Pitch Control (Elevator)

* L1 / R1 (Buttons 9 & 10): Throttle Decrease / Increase

* Cross (X): Toggle Autopilot (Holds current Mach and Altitude)

* Circle (O): Toggle Speedbrake

* Triangle (Δ): Toggle Landing Gear

**Keyboard**
* W / S (or Up / Down): Pitch Down / Pitch Up

* L-SHIFT / L-CTRL: Throttle Increase / Decrease

* A: Toggle Autopilot

* B: Toggle Speedbrake

* G: Toggle Landing Gear

## 📊 HUD Telemetry Guide
The PyGame terminal acts as your primary flight display. It tracks:

**Base Kinematics:** Mach, Pitch, Angle of Attack (AoA), Flight Path Angle (Gamma), Altitude.

**Performance Metrics:** Specific Excess Power (SEP), Fuel Flow, Instantaneous Range, and Endurance.

**Systems:** Autopilot status, Afterburner engagement (triggers at >90% throttle), and Flight Control Computer (FCC) limiters.

---

## 👨‍💻 Author & Instructor

**Kıvanç Apaydın**
*Aeronautical Engineer | Flight Physics*
* [LinkedIn Profile](https://www.linkedin.com/in/k%C4%B1van%C3%A7-apayd%C4%B1n-3a53a8259/)

---

_© 2026 Applied Flight Performance Course. All rights reserved._
