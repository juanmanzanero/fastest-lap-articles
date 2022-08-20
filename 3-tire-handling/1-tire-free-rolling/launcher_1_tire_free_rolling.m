clear all
clc
close all

%filename='1_tire_free_rolling.gif'; omega = 1.0;
%filename='2_tire_positive_kappa.gif'; omega = 1.1;
%filename='3_tire_negative_kappa.gif'; omega = 1/1.1;
filename='4_tire_locked_kappa.gif'; omega = 0.0;
gif_duration = 4;
frame_rate = 30;
dtime = 1/frame_rate;

background_color =  [13/255, 17/255, 23/255];
text_color = [201,209,217]/255;
blue   = [0   0.447000000000000   0.741];
orange = [0.850000000000000   0.325000000000000   0.098000000000000];
theta = linspace(0,2*pi,100);
theta_checkp = linspace(-pi/2,3*pi/2,10);
theta_radius = linspace(-pi/2,3*pi/2,50);
t = linspace(0,2*pi/omega,gif_duration*frame_rate);

for i = 1:numel(t)
    fprintf('i: %d\n',i);
    h = figure('Visible','off');
    hold on
    plot([-1.1,8.0813],[0,0],'-','Color',text_color,'LineWidth',2)
    plot(theta_checkp+pi/2,zeros(1,numel(theta_checkp)),'o','Color',text_color,'MarkerFaceColor',text_color)
    theta_remaining_in_tire = linspace(-pi/2,3*pi/2-omega*t(i),100);
    
    plot(t(i)+cos(theta),1+sin(theta),'-','Color',text_color,'LineWidth',2)
    
    for i_radius = 1:numel(theta_radius)
        plot(t(i)+[0.9,1.0]*cos(theta_radius(i_radius)-omega*t(i)), 1+[0.9,1.0]*sin(theta_radius(i_radius)-omega*t(i)),'Color',text_color);
    end
        
    
    plot(t(i)+cos(theta_remaining_in_tire), 1+sin(theta_remaining_in_tire),'-','Color',orange,'LineWidth',2)
    plot([0,t(i)],[0,0],'-','Color',orange,'LineWidth',2)
    
    % Markers: plot on the tire those with theta > -pi/2
    theta_rotated = theta_checkp-omega*t(i);
    
    plot(t(i)+cos(theta_rotated(theta_rotated > -0.5*pi)),1+sin(theta_rotated(theta_rotated > -0.5*pi)),'o','Color',orange,'LineWidth',2,'MarkerFaceColor',orange);
    %plot(t(i)+pi/2+theta_rotated(theta_rotated < -0.5*pi),zeros(1,sum(theta_rotated < -0.5*pi)),'o','Color',orange,'LineWidth',2,'MarkerFaceColor',orange);
    
    road_checkpoints = (theta_checkp+pi/2)/omega;
    plot(road_checkpoints(road_checkpoints <= t(i)), zeros(1,sum(road_checkpoints <= t(i))),'o','Color',orange,'LineWidth',2,'MarkerFaceColor',orange);
    axis equal
    xlim([-1.1,8.0813])
    ylim([-0.1,2.1]);
    h.CurrentAxes.Visible = 'off';
    h.Color = background_color;
    h.Position(4) = 0.2*h.Position(2);
    % Export as GIF
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
    elseif i == numel(t)
        for j = 1 : 60
            imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',dtime);
        end
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',dtime);
    end
    close(h)
    
end