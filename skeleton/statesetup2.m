
function [state]=statesetup(state);

%settings=config('startup');

%% Create State

 state.alpha=2;   % Alpha [rad]
 state.betha=0;   % Beta  [rad]

 state.P=0;   % Roll angular velocity  [rad/s]
 state.Q=0;   % Pitch angular velocity [rad/s]
 state.R=0;   % Yaw angular velocity   [rad/s]

 state.adot=0;  % Angle of attack time derivative,  (Alpha_dot), [rad/s]
 state.bdot=0;  % Angle of sideslip time derivative, (Beta_dot), [rad/s]

% Different speedtypes - International units 

 state.AS=15;       % True airspeed [m/s]
 state.rho=1.225;
 state.ALT=0;       % Altitude [m] 
                            
