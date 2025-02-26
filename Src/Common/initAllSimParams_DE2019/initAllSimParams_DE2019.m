% Copyright 2021 Delft University of Technology
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%      http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License. 

function [act, base_windspeed, constr, DE2019, ENVMT, Lbooth, ...
    loiterStates, params, simInit, T, winchParameter] = initAllSimParams_DE2019(Kite_DOF)
% This function initializes the parameters for a complete kite simulation system, 
% taking into account environmental factors, aircraft constraints, and flight path parameters.
%
% The function follows these steps:
%
% 1. Loads the aircraft parameters from the DE2019 dataset.
% 2. Sets up environmental constants such as gravity and wind direction.
% 3. Initializes the simulation with time step, simulation duration, and logging intervals.
% 4. Defines constraints for the kite's motion, winch dynamics, and flight path.
% 5. Configures tether parameters based on material properties and maximum force.
% 6. Initializes control system parameters for aircraft maneuvering during different phases.
% 7. Sets loitering state parameters for the power cycle initialization.
%
% **Function Breakdown**:
% This function is divided into several distinct sections that each serve a specific purpose:
%
% 1. **Aircraft Parameters Initialization**:
%    - Loads the DE2019 aircraft parameters and constraints from the `DE2019_params.mat` file.
%
% 2. **Environmental Parameters Setup**:
%    - Defines key constants like gravity (`ENVMT.g`), air density (`ENVMT.rhos`), and wind direction (`ENVMT.windDirection_rad`).
%    - Loads wind data (`winddata_Lidar_Avg.mat`) and calculates the wind speed at the maximum altitude.
%
% 3. **Simulation Initialization**:
%    - Initializes simulation parameters like simulation time (`simInit.TSIM`), time step (`simInit.dt`), and various logging sample times for force, speed, and position.
%    - Sets up initial conditions for aircraft position and attitude, converting between wind frame and object frame.
%
% 4. **Aircraft Constraints**:
%    - Defines constraints on the aircraft's motion, such as maximum angles (`mu_a_max`), lift coefficients (`max_lift`), and velocity limits.
%    - Specifies constraints for both the traction (power generation) and retraction (retrieving the kite) phases, as well as winch speed and acceleration limits.
%
% 5. **Tether Parameters**:
%    - Calculates the tether diameter and linear density based on the maximum tether force, material properties, and safety factors.
%    - Initializes the tether's physical properties, including stiffness (elastic modulus) and cross-sectional area.
%    - Sets up the initial length and discretization of the tether.
%
% 6. **Winch Parameters**:
%    - Defines the parameters of the winch system, such as initial reel-out speed, radius, inertia, and friction. 
%    - Sets the initial angular position of the winch drum based on tether length.
%
% 7. **Flight Path Parameters**:
%    - Defines the flight path geometry, including parameters like the size, shape, and elevation of the flight path.
%    - Specifies limits on the tether length during flight and the elevation angle at which transitions occur between flight phases.
%
% 8. **Control Parameters**:
%    - Sets the gains for the controllers in different flight phases (traction, retraction, loitering). These parameters determine the control behavior for maintaining position, adjusting speed, and correcting flight path errors.
%    - The `params` structure also contains values for controlling angles, speed, and winch behavior during various phases.
%
% 9. **Loitering State Parameters**:
%    - Specifies the parameters for the loitering state, including desired velocity, control gains for altitude and course, and a radius for the loitering path.
%    - Defines valid angular ranges for transitioning between loitering and other operational phases.
% :param Kite_DOF: Degrees of freedom of the kite (3 or 6), which determines the complexity of the system.
% :type Kite_DOF: integer
%
% :returns:
%
%   - **act** (*struct*): Actuator parameters, including aileron, elevator, and rudder data.
%   - **base_windspeed** (*double*): Wind speed at maximum altitude, where the speed stays constant.
%   - **constr** (*struct*): Aircraft maneuver and winch constraints.
%   - **ENVMT** (*struct*): Environmental parameters, including gravity and air density.
%   - **Lbooth** (*struct*): Flight path parameters, including size and shape of the flight path.
%   - **loiterStates** (*struct*): Initial loiter parameters for power cycle initialization.
%   - **DE2019** (*struct*): Aircraft parameters, including wing lift characteristics.
%   - **simInit** (*struct*): Simulation initialization parameters, including time step and logging sample times.
%   - **T** (*struct*): Tether dimensions and material properties.
%   - **winchParameter** (*struct*): Winch dynamic parameters, including reel-out speed and friction.
%   - **params** (*struct*): Flight and winch controller parameters, including control gains and limits.
%
% **Example**:
%
% .. code-block:: matlab
%
%    [act, base_windspeed, constr, ENVMT, ...
%          Lbooth, loiterStates, DE2019, simInit, ...
%          T, winchParameter, params] = initAllSimParams_DE2019(Kite_DOF);
%
% **Other Required Files**:
%   - transformFromWtoO.m
%   - transformFromOtoW.m
%   - getPointOnBooth.m
%
% **MAT-files**:
%   - Lib/Common/DE2019_params.mat
%   - Lib/6DoF/Control_allocation_V60.mat
%   - Lib/Common/winddata_Lidar_Avg.mat
%
%
% :Revision: 17-February-2025
% :Author: YuanHao Cui (yuanlidh@mail.dlut.edu.cn)


%------------- BEGIN CODE --------------
%% Obtain large-scale wing params
load('DE2019_params.mat','DE2019','constraintOut');

%% Environment struct
ENVMT.g = 9.8066;
ENVMT.rhos = 1.2250;
ENVMT.windDirection_rad = 3.1416;

load('winddata_Lidar_Avg.mat', 'h', 'vw_norm_metmast_ijmuiden')
% load('winddata_Lidar_Avg.mat', 'h', 'vw_norm_metmast_cabauw')

ENVMT.wind_height = h;
ENVMT.wind_data = vw_norm_metmast_ijmuiden;
ENVMT.height_max = h(ENVMT.wind_data==max(ENVMT.wind_data)); % keep wind speed constant from this height 
% 250m for ijmuiden, 500m for cabauw

%% Max winspeed measured at max altitude
base_windspeed = 22; %

%% Initialization parameters
simInit.TSIM = 1200; % Maximum simulation time 
simInit.dt = 0.005; % Fixed-step size
simInit.FSI_switchtime = 10000; % switching turned off because there is no active FSI

simInit.FtLoggingTs = 0.01; %Tether force logging sample time
simInit.vWLoggingTs = 0.01; %Wind speed logging sample time
simInit.CteLoggingTs = 0.2; %cross-track error logging sample time
simInit.AttLoggingTs = 0.5; %Attitude (alpha, beta) logging sample time
simInit.GSLoggingTs = 0.5;  %Glide slope error logging sample time
simInit.PLoggingTs = 0.01; %Power logging sample time, others use the same
simInit.Ts_power_conv_check = 0.1; %Power convergence check sample time

% Aircraft position and attitude
pos_init_W =  [200;400;450];
simInit.pos_O_init = transformFromWtoO(ENVMT.windDirection_rad, pos_init_W)'; %[-200,0,-250];
simInit.vel_B_init = [50;0;0];
psi_init = ENVMT.windDirection_rad+pi;
if psi_init > pi
    psi_init = ENVMT.windDirection_rad-pi;
end
simInit.eta_init = [0;0;psi_init];% Theta = theta_,
simInit.pos_W_init = transformFromOtoW(ENVMT.windDirection_rad, simInit.pos_O_init');
simInit.long_init = atan2( simInit.pos_W_init(2), simInit.pos_W_init(1) );
simInit.lat_init = asin( simInit.pos_W_init(3)/norm(simInit.pos_W_init) );

%% Constrainst
constr.mu_a_max = deg2rad(60);
constr.mu_a_min = -constr.mu_a_max;

alphawing = deg2rad(DE2019.initAircraft.alpha)';
liftwing = DE2019.initAircraft.wing_cL_Static';
[~, ind] = unique(liftwing);

constr.alpha_a_max = alphawing(ind(end));
constr.alpha_a_min = deg2rad(-15);

constr.mu_a_max_retraction = 0.7854;
constr.gamma_min_retraction = -40; %degrees
constr.gamma_max_retraction = 30; %degrees
% Maximum lift generated by aircraft
constr.max_lift = constraintOut.buckling;

constr.max_delta_mu = 0.1745;
constr.mu_k_max = 1.0472;
if Kite_DOF == 6
    constr.max_rates_B = deg2rad(50);
end

% Maximum tether force 
constr.F_T_max = 0.9*constr.max_lift;

constr.Va_min = 15; % Minimum velocity before engines turn on to assist
constr.max_CL = 2.5; % Hard code maximum allowable cl
constr.min_CL = -2.5; % Hard code minimum allowable cl
constr.min_altitude_pc = 400; % Retraction target minimal altitude
constr.winchParameter.a_max = 5;
constr.winchParameter.a_min = -5;  % winch acceleration limits 
constr.winchParameter.v_max = 25; % winch reelout speed limits.
constr.winchParameter.v_min =-25;

%% Tether parameters
SFt = 1.5; % Safety Factor
sigmat = 3.6*10^9; % % Kite power fact sheet: Dyneema® high-strength, high-modulus polyethylene fiber
rho_dyneema = 970; %kg/m3 % Kite power fact sheet: Dyneema® high-strength, high-modulus polyethylene fiber

T.d_tether = sqrt(constr.F_T_max/(sigmat/(4/pi*SFt)));
T.rho_t = rho_dyneema * (pi/4*T.d_tether^2); %kg/m 
T.CD_tether = 1.2;

T.E = 116e9; % Kite power fact sheet: Dyneema® high-strength, high-modulus polyethylene fiber
T.A = (pi/4*T.d_tether^2);
T.np = 15; %number of descretization particles, needs convergence study

% DE2019.rho_air = 1.2250;

l_j = norm(simInit.pos_W_init);
phi_init = atan2(simInit.pos_W_init(2),sqrt(l_j^2-simInit.pos_W_init(2)^2));
if abs(phi_init)<1e-5
    phi_init = 0;
end
theta_init = atan2(simInit.pos_W_init(1),simInit.pos_W_init(3));

T.Tn_vect = [theta_init; phi_init; 0.002*T.E*T.A]; %theta,phi,force magnitude

T.tether_inital_lenght = norm( simInit.pos_O_init);
T.l0 = T.tether_inital_lenght/(T.np+1);
T.pos_p_init = [];
e_t = simInit.pos_W_init/norm(simInit.pos_W_init);
for p = 1 : T.np
    T.pos_p_init = [T.pos_p_init, p*e_t*T.l0];
end
T.pos_p_init = fliplr(T.pos_p_init);

T.eta1 = 0.403362463949762;
T.eta2 = 0.646914366052548;
T.alpha1 = 4.33961097083308;
T.alpha2 = 0.745653460592095;

%% Winch parameters
%Needs further research at this scale
winchParameter.lj_dot_init = 0; %initial reel-out speed
winchParameter.radius = 0.4;
winchParameter.inertia = 32;
winchParameter.friction = 10; %dynamic friction
winchParameter.winch_angle_init = T.tether_inital_lenght/winchParameter.radius;

%% Flightpath parameter 
params.a_booth = 0.9;
params.b_booth = 500;
params.phi0_booth = 0.523599;

params.l_tether_max = 1400;
params.l_tether_min = 0.8;

params.initial_path_elevation = 30;
params.w0_decrease_init_phi0  = 0.1;
params.transition_phi = 30; % set the elevation of the transition figure

%% Tetherforce set point
params.F_t_traction_set = 1.37659e+06;
params.F_t_retraction_set = 0.213972;
loiterStates.Ft_set_loiter = 1e6; %During initialisation

% params.ft_smooth_1= 2.5000e-04; % Tether set point slow increase parameter (state machine)
params.ft_smooth_2 = 0.0893862; % Tether set point slow increase parameter (state machine)
% params.ft_smooth_3=1e-2; % Tether set point slow increase parameter (state machine)

%% Control parameters

params.Kp_chi_k_traction = 0.632409;
params.Kp_gamma_k_traction = 0.918834;

params.Kp_chi_retraction = 0.31442;
params.Kp_gamma_retraction = 0.33232;
params.Ki_chi_retraction = 0;
params.Ki_gamma_retraction = 0;

params.Kp_chi_loiter = 6.80238;
params.Kp_gamma_loiter = 0.005;

params.Kp_mu = 0.05;
params.Kp_alpha = 0.451566;

params.Ki_mu = 0; % commented out in simulink
params.Ki_alpha = 0.1000;

params.kp_winch = 0.96871;
params.ki_winch = 0;

params.crosstrackerror_condition = 400;
params.w0SpeedControl = 0.5000;
params.kp_speed_control = 5;
params.ki_speed_control = 5;
params.Kp_brake = 0.43288;
params.Ki_brake = 0.106604;

params.r_retraction = 100;
params.kp_delta_course = 0.1;
params.kp_delta_gamma = 0.05;
% params.v_k_min= 10;
% params.dvk= 2;
params.delta0_offset = 0.1500;
params.delta0_slope = 0;

params.w0_chi_retraction_max = 0.05;
params.w0_gamma_retraction_max = 1;
params.w0_mu = 6;
params.w0_alpha = 7;

if Kite_DOF==6
    params.Kp_beta = .1;
    params.Ki_beta = 0; % commented out in simulink
    params.Kp_p = 7.82054;
    params.Kp_q = 10;
    params.Kp_r = 10;
    params.w0_p_max = 8.34278;
    params.w0_q_max = 5.95005;
    params.w0_r_max = 15;
end

%Retraction phase
params.arc1 = 3.5; % Minimum: >1 determines size arc radius
params.arcPoints = 10; %discretisation points for the retraction phase navigation flight path
params.x_arcEnd = 0;
% params.arc_phi =0; %Inclination of retraction phase 2 in degrees

%% Parameters of the loitering state (initialisation phase)
loiterStates.Va_set = 47.4746;
loiterStates.kp_h = 0.1000;
loiterStates.kp_delta_course = 0.05;

loiterStates.R = 200;
loiterStates.pos_OLoiter_W = [100;0;200];
loiterStates.validTransReg_start = -3.1416; %-pi
loiterStates.validTransReg_end = -1.3963; %-11*pi/25

%% Actuators
if Kite_DOF==6
    % Aileron
    act.aileron.w0 = 35;
    act.aileron.rl = 2;%300*pi/180; %
    act.aileron.max_deflection = 1; %+-35º
    act.aileron.min_deflection = -1;
    act.aileron.bias = 0;

    % Elevator
    act.elevator.w0 = 35; %
    act.elevator.rl = 2;%300*pi/180; %
    act.elevator.max_deflection = 19*pi/180;
    act.elevator.min_deflection = -19*pi/180;
    act.elevator.bias = 0;

    % Rudder
    act.rudder.w0 = 35; %
    act.rudder.rl = 2;%300*pi/180; %
    act.rudder.max_deflection = 19*pi/180;
    act.rudder.min_deflection = -19*pi/180;
    act.rudder.bias = 0;
else
    act = 'No actuators modelled';
end
%% Control allocation matrix data generation
% Control allocation matrix calculated at different angles of attack based
% on DE2019 aircraft.
if Kite_DOF==6
    load('Control_allocation_V60.mat','alphalist','B_A11','B_A21','B_A31',...
        'B_A22','B_A13','B_A33')

    B_A11f = fit(alphalist',B_A11','poly2');
    B_A21f = fit(alphalist',B_A21','poly2'); %almost linear but poly2 gives lower RMSE
    B_A31f = fit(alphalist',B_A31','poly2');
    B_A22f = fit(alphalist',B_A22','poly2');
    B_A13f = fit(alphalist',B_A13','poly2');
    B_A33f = fit(alphalist',B_A33','poly2');

    DE2019.Cl_deltaA_0 = B_A11f.p3;
    DE2019.Cl_deltaA_alpha = B_A11f.p2;
    DE2019.Cl_deltaA_alpha2 = B_A11f.p1;
    DE2019.Cl_deltaR_0 = B_A13f.p3;
    DE2019.Cl_deltaR_alpha = B_A13f.p2;
    DE2019.Cl_deltaR_alpha2 = B_A13f.p1;

    DE2019.Cm_deltaA_0 = B_A21f.p3;
    DE2019.Cm_deltaA_alpha = B_A21f.p2;
    DE2019.Cm_deltaA_alpha2 = B_A21f.p1;
    DE2019.Cm_deltaE_0 = B_A22f.p3;
    DE2019.Cm_deltaE_alpha = B_A22f.p2;
    DE2019.Cm_deltaE_alpha2 = B_A22f.p1;

    DE2019.Cn_deltaA_0 = B_A31f.p3;
    DE2019.Cn_deltaA_alpha = B_A31f.p2;
    DE2019.Cn_deltaA_alpha2 = B_A31f.p1;
    DE2019.Cn_deltaR_0 = B_A33f.p3;
    DE2019.Cn_deltaR_alpha = B_A33f.p2;
    DE2019.Cn_deltaR_alpha2 = B_A33f.p1;
end
%% Lemniscate parameter initialisation, no need to be changed
Lbooth.a = 0; %initialisation of variable
Lbooth.b = 0; %initialisation of variable
Lbooth.phi0 = 0; %initialisation of variable
Lbooth.init_sol = 3.9270; %5*pi/4
%------------- END CODE --------------
end
