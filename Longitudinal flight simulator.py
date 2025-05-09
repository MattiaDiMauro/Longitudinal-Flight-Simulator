# Longitudinal Flight Dynamics Simulator
# Author: Mattia Di Mauro
# Year: 2025

# This Python script simulates the longitudinal flight dynamics of two fixed-wing aircraft:
# a light general aviation aircraft (North American L-17 Navion) and a large commercial airliner (Boeing 747).

# The simulator builds linearized state-space models from given aerodynamic, inertial and geometric parameters,
# and solves the time response to user-defined elevator deflection inputs (with thrust assumed constant).
# It essentially allows the user to analyze the different dynamic responses of two aircraft to the same elevator input.

# The goal of this project is purely educational and personal (to improve my programming skills in Python).

#Note: This simulator does not serve any academic or professional purpose and is not validated for engineering use.

import numpy as np
import matplotlib.pyplot as plt
import math

g = 9.80665
theta_0 = 0  

# Conversion
ft_to_m = 0.3048
slug_ft2_to_kgm2 = 1.35581795  
lbs_to_kg = 0.45359237

# Geometric Data (Navion)
mass = 2750 * lbs_to_kg  
wing_surface = 184 * ft_to_m**2  
chord = 5.7 * ft_to_m  
span = 33.4 * ft_to_m  
Iyy = 3000 * slug_ft2_to_kgm2  
MaxThrust = 71000  

# Flight condition (Navion)
Mach = 0.158
rho = 2.3769e-03 * 515.379  
speed_of_sound = math.sqrt(1.4 * 287.05 * 288.15)  
U_0 = speed_of_sound * Mach  

# Aerodynamic data (Navion)
CL_0 = 0.41
CL_alpha = 4.44
CL_alphadot = 0
CL_q = 3.8
CL_deltat = 0.0
CL_deltae = 0.355
CL_u = 0

CD_0 = 0.05
CD_alpha = 0.33
CD_alphadot = 0.0
CD_q = 0.0
CD_deltat = -1.0
CD_deltae = 0.0
CD_u = 0.0 * Mach

Cm_0 = 0.0
Cm_alpha = -0.683
Cm_alphadot = -4.36
Cm_q = -9.96
Cm_deltat = 0.0
Cm_deltae = -0.923
Cm_u = 0

# Dimensionless parameters
m_1 = mass / (0.5 * rho * U_0 * wing_surface)
c_1 = chord / (2 * U_0)
Jy_1 = Iyy / (0.5 * rho * U_0**2 * wing_surface * chord)

# Stability and control derivatives
CX_alphadot = -CD_alphadot
CZ_alphadot = -CL_alphadot
CM_alphadot = Cm_alphadot

CX_u = -2 * CD_0 - CD_u
CX_alpha = -CD_alpha + CL_0
CX_q = -CD_q

CZ_u = -2 * CL_0 - CL_u
CZ_alpha = -CL_alpha - CD_0
CZ_q = -CL_q

CM_u = 2 * Cm_0 + Cm_u
CM_alpha = Cm_alpha + Cm_0
CM_q = Cm_q

CX_deltat = -CD_deltat
CX_deltae = -CD_deltae
CZ_deltat = -CL_deltat
CZ_deltae = -CL_deltae
CM_deltat = Cm_deltat
CM_deltae = Cm_deltae

# Matrices Navion

M_longNavion = np.array([
    [m_1, 0, 0, 0],
    [0, -c_1 * CZ_alphadot + m_1, 0, 0], 
    [0, -c_1 * CM_alphadot, Jy_1, 0],
    [0, 0, 0, 1]
])

K_longNavion = np.array([
    [-CX_u, -CX_alpha, 0, m_1 * (g/U_0) * np.cos(theta_0)],
    [-CZ_u, -CZ_alpha, -c_1 * CZ_q - m_1, m_1 * (g/U_0) * np.sin(theta_0)], 
    [-CM_u, -CM_alpha, -c_1 * CM_q, 0],
    [0, 0, -1, 0]
])

Delta_longNavion = np.array([
    [CX_deltat, CX_deltae],
    [CZ_deltat, CZ_deltae], 
    [CM_deltat, CM_deltae],
    [0, 0]
])

# Geometric Data (B747)
mass = 636600 * lbs_to_kg  
wing_surface = 5500 * ft_to_m**2  
chord = 27.31 * ft_to_m  
span = 195.68 * ft_to_m  
Iyy = 33.10*(10**6) * slug_ft2_to_kgm2  
MaxThrust = 71000 

# Flight condition (B747)
Mach = 0.25
rho = 2.3769e-03 * 515.379  
speed_of_sound = math.sqrt(1.4 * 287.05 * 288.15)  
U_0 = speed_of_sound * Mach  

# Aerodynamic data (B747)
CL_0 = 1.11
CL_alpha = 5.7
CL_alphadot = 6.7
CL_q = 5.4
CL_deltat = 0.0
CL_deltae = 0.338
CL_u = -0.81 * Mach

CD_0 = 0.102
CD_alpha = 0.66
CD_alphadot = 0.0
CD_q = 0.0
CD_deltat = -1.0
CD_deltae = 0.0
CD_u = 0.0 * Mach

Cm_0 = 0.0
Cm_alpha = -1.26
Cm_alphadot = -3.2
Cm_q = -20.8
Cm_deltat = 0.0
Cm_deltae = -1.34
Cm_u = 0.27 * Mach

# Dimensionless parameters
m_1 = mass / (0.5 * rho * U_0 * wing_surface)
c_1 = chord / (2 * U_0)
Jy_1 = Iyy / (0.5 * rho * U_0**2 * wing_surface * chord)

# Stability and control derivatives
CX_alphadot = -CD_alphadot
CZ_alphadot = -CL_alphadot
CM_alphadot = Cm_alphadot

CX_u = -2 * CD_0 - CD_u
CX_alpha = -CD_alpha + CL_0
CX_q = -CD_q

CZ_u = -2 * CL_0 - CL_u
CZ_alpha = -CL_alpha - CD_0
CZ_q = -CL_q

CM_u = 2 * Cm_0 + Cm_u
CM_alpha = Cm_alpha + Cm_0
CM_q = Cm_q

CX_deltat = -CD_deltat
CX_deltae = -CD_deltae
CZ_deltat = -CL_deltat
CZ_deltae = -CL_deltae
CM_deltat = Cm_deltat
CM_deltae = Cm_deltae

# Matrices B747

M_longB = np.array([
    [m_1, 0, 0, 0],
    [0, -c_1 * CZ_alphadot + m_1, 0, 0], 
    [0, -c_1 * CM_alphadot, Jy_1, 0],
    [0, 0, 0, 1]
])

K_longB = np.array([
    [-CX_u, -CX_alpha, 0, m_1 * (g/U_0) * np.cos(theta_0)],
    [-CZ_u, -CZ_alpha, -c_1 * CZ_q - m_1, m_1 * (g/U_0) * np.sin(theta_0)], 
    [-CM_u, -CM_alpha, -c_1 * CM_q, 0],
    [0, 0, -1, 0]
])

Delta_longB = np.array([
    [CX_deltat, CX_deltae],
    [CZ_deltat, CZ_deltae], 
    [CM_deltat, CM_deltae],
    [0, 0]
])

A_matrixNavion = np.linalg.solve(-M_longNavion, K_longNavion)
B_matrixNavion = np.linalg.solve(M_longNavion, Delta_longNavion)
A_matrixB = np.linalg.solve(-M_longB, K_longB)
B_matrixB = np.linalg.solve(M_longB, Delta_longB)

Durationsimulation = int(input("Enter the duration of the entire response simulation (in seconds): "))

# Elevator inputs
elevator_pulses = []

elevatornumber = int(input("Number of elevator inputs: "))
for i in range(elevatornumber):
    elevatorduration = float(input(f"[Input {i+1}] Duration of the elevator input (sec): "))
    elevatorstarting = float(input(f"[Input {i+1}] Starting time of the elevator input (sec): "))
    elevatorangle = float(input(f"[Input {i+1}] Angle of the elevator input (deg): "))
    elevator_pulses.append((elevatorstarting, elevatorstarting + elevatorduration, elevatorangle))

def elevator_input(t):
    for start, end, angle in elevator_pulses:
        if start <= t < end:
            return angle * np.pi / 180
    return 0

# Thrust input (null)
def thrust_input(t):
    for i in range(Durationsimulation):
        return 0

# Response simulation 
def simulate_responseNav(t_span, y0):
    from scipy.integrate import solve_ivp

    def state_derivativeNav(t, y):
        u, alpha, q, theta = y

        # Input at time t
        delta = np.array([thrust_input(t), elevator_input(t)])
        
        # Systems
        y_dotNavion = A_matrixNavion @ y + B_matrixNavion @ delta
        return y_dotNavion
        
    # Response computation
    solutionNav = solve_ivp(
        lambda t, y: state_derivativeNav(t, y),
        t_span,
        y0,
        method='RK45',
        t_eval=t_evalgeneric,
        rtol=1e-8,
        atol=1e-8
    )
    return solutionNav.t, solutionNav.y

def simulate_responseB(t_span, y0):
    from scipy.integrate import solve_ivp

    def state_derivativeB(t, y):
        u, alpha, q, theta = y
        
        # Input at time t
        delta = np.array([thrust_input(t), elevator_input(t)])
        
        # Systems
        y_dotB = A_matrixB @ y + B_matrixB @ delta
        return y_dotB
    
    # Response computation
    solutionB = solve_ivp(
        lambda t, y: state_derivativeB(t, y),
        t_span,
        y0,
        method='RK45',
        t_eval=t_evalgeneric,
        rtol=1e-8,
        atol=1e-8
    )
    return solutionB.t, solutionB.y


    
# Initial state
y0 = np.array([0, 0.0, 0.0, 0.0])  # [u_0, alpha_0, q_0, theta_0]

# Response
t_span = (0, Durationsimulation)  
t_evalgeneric=np.linspace(t_span[0], t_span[1], 1000)
t, yNav = simulate_responseNav(t_span, y0)
t, yB = simulate_responseB(t_span, y0)


# Plots
plt.figure(figsize=(15, 10))

# Speed (u)
plt.subplot(3, 2, 1)
plt.plot(t, yNav[0], t, yB[0])
plt.title('Speed (u/U_0)')
plt.xlabel('Time (s)')
plt.grid(True)

# Angle of attack (alpha)
plt.subplot(3, 2, 2)
plt.plot(t, np.degrees(yNav[1]), t, np.degrees(yB[1]))
plt.title('Angle of attack (alpha)')
plt.xlabel('Time (s)')
plt.ylabel('Degree')
plt.grid(True)

# Angular velocity (q)
plt.subplot(3, 2, 3)
plt.plot(t, np.degrees(yNav[2]), t, np.degrees(yB[2]))
plt.title('Angular velocity (q)')
plt.xlabel('Time (s)')
plt.ylabel('Degree/s')
plt.grid(True)


# Pitch angle (theta)
plt.subplot(3, 2, 4)
plt.plot(t, np.degrees(yNav[3]), t, np.degrees(yB[3])) 
plt.legend(["L-17 Navion", "Boeing 747"], loc='upper center', bbox_to_anchor=(0.5, -0.3), fancybox=True, shadow=True)
plt.title('Pitch angle (theta)')
plt.xlabel('Time (s)')
plt.ylabel('Degree')
plt.grid(True)

# Elevator inputs
elevator_inputs = np.array([elevator_input(time) for time in t])
plt.subplot(3, 2, 5)
plt.plot(t, np.degrees(elevator_inputs))
plt.title('Elevator input')
plt.xlabel('Time (s)')
plt.ylabel('Degree')
plt.grid(True)

plt.tight_layout()
plt.savefig('b747_longitudinal_response.png')
plt.show()