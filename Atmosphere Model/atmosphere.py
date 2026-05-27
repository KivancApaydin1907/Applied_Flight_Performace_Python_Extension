import math

def atmosisa(h):
    """
    1976 Standard Atmosphere Model
    Calculates temperature, speed of sound, pressure, and density.
    
    Inputs:
        h: Altitude [m] (Geometric)
    Outputs:
        T: Temperature [K]
        a: Speed of Sound [m/s]
        P: Pressure [Pa]
        rho: Density [kg/m^3]
    """
    # Standard Constants
    R = 287.0528      # Gas constant [J/(kg K)]
    gamma = 1.4       # Specific heat ratio
    g = 9.80665       # Gravity [m/s^2]
    
    # Boundary Conditions at Sea Level
    T0 = 288.15       # [K]
    P0 = 101325.0     # [Pa]
    L = -0.0065       # Lapse rate [K/m]
    
    # Troposphere (0 to 11,000 m)
    if h <= 11000.0:
        T = T0 + L * h
        P = P0 * (T / T0)**(-g / (L * R))
        
    # Lower Stratosphere (11,000 m to 20,000 m)
    else:
        T11 = 216.65  # Temperature at 11km [K]
        P11 = 22632.1 # Pressure at 11km [Pa]
        T = T11
        # Isothermal pressure equation
        P = P11 * math.exp(-g * (h - 11000.0) / (R * T11))
        
    rho = P / (R * T)
    a = math.sqrt(gamma * R * T)
    
    return T, a, P, rho
