clear all
clc
close all

cd = 0.99642659990346749;
cl = 2.7101908445246665;

Area = 1.5;
rho = 1.225;

% Data according to https://maxtayloraero.com/2022/01/17/aerodynamic-studies-of-a-2022-f1-car/
cd_rw_percentage_1 = 0.337/1.503;
cl_rw_percentage_1 = 1.042/3.679;

% Data according to https://www.f1technical.net/forum/viewtopic.php?t=30329
cd_rw_percentage_2 = 553/2183;
cl_rw_percentage_2 = (1693)/10876;

% Old position of the center of pressure:
x_cg = 1.5216093891550388;
x_cp_old = x_cg - 0.092900898328350232;
x_cp_rw = -0.5;
cl_no_drs = cl*(1-cl_rw_percentage_2);
cl_rw = cl*cl_rw_percentage_2;
% cl.x_cp_old = cl_no_drs.x_cp_new + cl_rw.x_cp_rw
x_cp_new = (cl*x_cp_old - cl_rw*x_cp_rw)/cl_no_drs;

cd_no_drs = cd*(1-cd_rw_percentage_2);

fprintf('New position of the center of pressure: %f\n',x_cp_new - x_cg);
fprintf('New cl: %f\n',cl_no_drs);
fprintf('New cd: %f\n',cd_no_drs);