# Core Physics Engine: `flight_dynamics.py`

This module serves as the central 3-Degree-of-Freedom (3-DOF) longitudinal equations of motion (EOM) solver for the T-38 Talon simulation. It computes the instantaneous aerodynamic forces, propulsion metrics, and kinematic derivatives required by the Runge-Kutta (RK4) time-integration loop.

## 🏗️ Architectural Shift: The Embedded Ground Constraint

A critical divergence in this Python implementation compared to traditional trajectory optimization setups (such as phase-based MATLAB/GPOPS scripts) is the **explicit, in-loop handling of ground physics**. 

In phase-based solvers, ground roll and free flight are often strictly separated into distinct mathematical phases with their own boundary conditions. However, for a real-time, continuous simulation, the physics engine must dynamically transition between free-flight EOMs and ground-constrained EOMs without solver interruption.

To achieve this, the ground constraint is embedded directly into the dynamics function:
1. **Normal Force Calculation:** The engine continuously calculates the theoretical normal force required to prevent the aircraft from sinking into the Earth frame.
2. **Kinematic Lock:** If the aircraft is at ground level ($z_E \ge 0$) and the normal force is positive ($N_{force} > 0$), the engine applies Coulomb friction ($F_{friction} = \mu \cdot N_{force}$) and completely locks the Earth-frame vertical velocity ($z_{dot\_E} = 0$). 
3. **Body-Frame Derivation:** It then reverse-calculates the necessary body-frame accelerations ($\dot{u}$ and $\dot{w}$) to ensure the flight path angle remains perfectly tangential to the runway, preventing numerical bouncing or integration singularities upon touchdown.

---

## 🧮 Subsystem Breakdown

### 1. Kinematics & Coordinate Transformations
The engine tracks variables in the Body Frame ($u, w$) and translates them to the Earth Frame ($x_E, z_E$) and Stability Frame using standard Direction Cosine Matrices (DCM). 
* **Angle of Attack ($\alpha$):** $\arctan(w/u)$
* **Flight Path Angle ($\gamma$):** $\theta - \alpha$
* Dynamic pressure ($\bar{q}$) and Mach number are derived continuously based on the standard atmosphere inputs ($\rho, a$).

### 2. Aerodynamics Model
Aerodynamic coefficients are processed using a hybrid approach of localized linear derivatives and non-linear lookups:
* **Lift ($C_L$):** Base lift is interpolated via lookup tables $f(M, \alpha)$, then modified by linear elevator control derivatives ($C_{L_{\delta_e}}$).
* **Drag ($C_D$):** Calculated using a standard drag polar formulation ($C_{D_0} + K \cdot C_L^2$), supplemented by additive wave drag at transonic/supersonic regimes and configuration penalties (speedbrakes, gear).
* **Pitching Moment ($C_m$):** Handled linearly using $C_{m_0} + C_{m_\alpha}\alpha + C_{m_{\delta_e}}\delta_e$.

### 3. Propulsion (J85-GE-5 Turbojets)
The engine model simulates the continuous performance of twin General Electric J85 turbojets, utilizing 2D lookup maps $f(h, M)$ for both thrust limits and Specific Fuel Consumption (SFC).
* **Dry vs. Wet Thrust:** The throttle input is mapped conditionally. Settings $\le 90\%$ interpolate strictly within the "Dry" performance maps. Pushing past $90\%$ engages the "Wet" (Afterburner) maps, blending the excess thrust and significantly increasing SFC.
* **Mass Derivation:** The fuel mass flow rate ($\dot{m}$) is continuously calculated and fed back into the state vector to update the aircraft's gross weight and inertia dynamically.

### 4. 3-DOF Equations of Motion (Free Flight)
When the ground constraint is inactive, the engine resolves the standard 3-DOF rigid body equations:
* **Axial Acceleration:** $\dot{u} = \frac{X}{m} - qw$
* **Normal Acceleration:** $\dot{w} = \frac{Z}{m} + qu$
* **Pitch Acceleration:** $\dot{q} = \frac{M_{aero}}{I_{yy}}$

---

## 📥 Inputs & 📤 Outputs

**Inputs:**
* `state`: The 7-element state vector (`x_E`, `z_E`, `u`, `w`, `theta`, `q`, `m`).
* `ac`: The aircraft database dictionary (geometry, mass, aero maps, engine maps).
* `env`: Atmospheric conditions ($\rho$, $a$, $g$, $\mu$).
* `controls`: Pilot/Autopilot inputs (`ElevatorDeflection`, `ThrottleSetting`, `SpeedBrake`, `Gear`).

**Outputs:**
A packaged dictionary returning:
* `Derivatives`: The rates of change for the RK4 integrator.
* `States` & `Forces`: Extracted telemetry (Mach, $\alpha$, $G$-load, Thrust) strictly for HUD rendering and logging.

<h2>👨‍💻 Author</h2>
<p><strong>Kıvanç Apaydın</strong> – Aerospace Engineer</p>
