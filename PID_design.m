% CONTROLAB - Advancing Control Systems Education
%
% Script Name: PID_design.m
% Version: 1.0
% Date: 17/10/2024
%
% Description:
% This script is part of Controlab's free control educational content.
% Here you will learn how to design a PID in continuous-time. Then, you
% will implement the discrete controller in the Simulink model using
% transfer function blocks and a coded MATLAB function. Have fun!
%
% Contact Information:
% Email: [contact_email@example.com]
% Website: [www.controlab.com.au]
%
% -------------------------------------------------------------------------

% clear workspace and terminal
clc; clear;


%% Configure Script

control_design = 0;  % 1: opens control designer.


%% System Model

% Plant parameters (Indutor)
L = 0.001;  % inductance
R = 0.1;    % coil parasitic resistance

% Create Plant Transfer function (inductor current)
s = tf('s');
Gvi = (1/R) / (s*L/R + 1);


%% Control Design

% open control system designer. Don't forget to export your PID controller!
if control_design
    rltool(Gvi);
end


%% Import Controller and Discretize

if exist('C.mat', 'file') == 2
    load('C.mat');

    % Discretization using different methods
    Ts = 0.5*10^(-3);   % controller sampling period
    C_tustin = c2d(C, Ts, 'tustin');

    C = tf(C)
    C_tustin = tf(C_tustin)

    % extract coefficients
    a2 = C_tustin.Numerator{1}(1);
    a1 = C_tustin.Numerator{1}(2);
    a0 = C_tustin.Numerator{1}(3);

    b2 = C_tustin.Denominator{1}(1);
    b1 = C_tustin.Denominator{1}(2);
    b0 = C_tustin.Denominator{1}(3);

else
    disp('ERROR: C.mat does not exist in the current folder.');
    disp('Remember to design your continuous-time controller and save it.');
end


%% Simulation
sim("model_simulation.slx");