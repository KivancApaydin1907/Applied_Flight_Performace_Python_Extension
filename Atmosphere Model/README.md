# 🌍 Standard Atmosphere Model: `atmosphere.py`

This module provides a lightweight, computationally efficient implementation of the **1976 U.S. Standard Atmosphere** model. It serves as the primary environmental truth source for the flight dynamics engine, calculating vital air properties as a function of geometric altitude.

## 🤝 MATLAB Parity (`atmosisa`)
If this function looks familiar, it is! This script was explicitly designed as a 1:1 Python port of MATLAB's built-in `atmosisa` function. 

For students transitioning from the theoretical MATLAB scripts in the Udemy course to this real-time Python simulation, this parity ensures that all derivative calculations, specifically dynamic pressure and Mach number, will match your earlier course results perfectly. There are no unexpected discrepancies introduced by the environment engine.

## 🧮 Atmospheric Layers Handled

The Northrop T-38 Talon has a maximum service ceiling of approximately 15,240 meters (50,000 ft). Therefore, this model accurately covers the two required atmospheric layers without wasting compute power on higher-altitude mesosphere logic:

* **The Troposphere (0 to 11,000 m):** Applies a standard temperature lapse rate of -0.0065 K/m. Pressure drops exponentially based on the shifting temperature.
* **The Lower Stratosphere (11,000 m to 20,000 m):** Models an isothermal layer where the temperature remains constant at 216.65 K. Pressure decays exponentially based strictly on the hydrostatic equation.

## 📥 Inputs & 📤 Outputs

**Input:**
* `h`: Geometric altitude in meters. (Note: The physics engine automatically clamps this to a minimum of 0.0 to prevent subterranean atmosphere calculations during runway operations).

**Outputs:**
* `T`: Static Temperature in Kelvin.
* `a`: Local Speed of Sound in m/s *(Critical for continuous Mach number derivation).*
* `P`: Static Pressure in Pascals.
* `rho`: Air Density in kg/m^3 *(Critical for calculating Dynamic Pressure and subsequent Lift/Drag forces).*

<h2>👨‍💻 Author</h2>
<p><strong>Kıvanç Apaydın</strong> – Aerospace Engineer</p>
