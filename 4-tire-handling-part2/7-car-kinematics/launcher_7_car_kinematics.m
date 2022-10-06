clear all
clc
close all

% Script inputs
font_name              = 'Formula1 Display Regular';
car_name               = 'ferrari';
frame_rate             = 15;
final_time             = 5;


set(groot,'defaultAxesFontName',font_name);
dtime = 1/frame_rate;
time = 0:dtime:final_time;


h = figure;

h_im = plot_f1(0,0,0,3.6,0.73*2,car_name);
for i = 1 : numel(time)
end
