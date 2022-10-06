clear all
clc
close all

global text_color background_color


gif_duration = 4;
frame_rate = 30;
dtime = 1/frame_rate;
filename = 'tire_lateral_3d.gif';

%background_color =  [13/255, 17/255, 23/255];
%text_color = [201,209,217]/255;
text_color =  [13/255, 17/255, 23/255];
background_color = [1,1,1];

blue   = [0   0.447000000000000   0.741];
orange = [0.850000000000000   0.325000000000000   0.098000000000000];

t = 0:dtime:2*pi;

for i_time = 1:numel(t)
    fprintf('%d/%d\n',i_time,numel(t))
    
    x = 2*pi-t(i_time);
    y = 0.2*(2*pi-x);
    plot_frame(x,y,1.0,t(i_time))

    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i_time == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
    elseif i_time == numel(t)
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',2);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',dtime);
    end
    close(gcf)
end

t = 0:dtime:4;
for i_time = 1:numel(t)
    fprintf('%d/%d\n',i_time,numel(t))
    
    x = 0.0;
    y = 0.2*(2*pi)*exp(-3*t(i_time)).*cos(20*t(i_time));
    plot_frame(x,y,1.0,2*pi)

    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);

    if i_time == numel(t)
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',1);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',dtime);
    end
    close(gcf)
end

function plot_frame(x_center, y_center, omega_center, time)
global text_color background_color

h = figure('Visible','off');
h.Color = background_color;
h.Position = [340         269        1226         623];
hold all
h.CurrentAxes.Visible = 'off';
h.CurrentAxes.Position = [0,0,1,1];
theta_center = omega_center * time;


% (1) Plot the grid
x_grid = -5:10;
y_grid = -2:4;

grid_lines_width = 0.025;
patch([x_grid(1),x_grid(end),x_grid(end),x_grid(1),x_grid(1)],[y_grid(1),y_grid(1),y_grid(end),y_grid(end),y_grid(1)],zeros(1,5),background_color);

for i = 1 : numel(x_grid)
    plot3([x_grid(i),x_grid(i)],[y_grid(1),y_grid(end)],[0,0],'color',0.2*text_color)
end
for i = 1 : numel(y_grid)
    plot3([x_grid(1),x_grid(end)],[y_grid(i),y_grid(i)],[0,0],'color',0.2*text_color)
end


% (2) Plot the metal sheet
n_metal_sheet = 500;
n_metal_sheet_flat = 100;
metal_sheet_width = 0.35;
sheet_color = '#6F8FAF';


theta_sheet_center = linspace(theta_center-pi/2,3*pi/2,n_metal_sheet);

max_length_sheet_center = 2*pi/omega_center;

length_sheet_center = min(2*pi-x_center, max_length_sheet_center);

slope = y_center/(1.0e-12+2*pi-x_center);
y_metal_sheet_end = slope*length_sheet_center;

cMap = jet;

surf([linspace(2*pi-length_sheet_center,2*pi,n_metal_sheet_flat);linspace(2*pi-length_sheet_center,2*pi,n_metal_sheet_flat)],...
    [linspace(y_metal_sheet_end,0,n_metal_sheet_flat) + metal_sheet_width;linspace(y_metal_sheet_end,0,n_metal_sheet_flat) - metal_sheet_width],...
    zeros(2,n_metal_sheet_flat),[linspace(abs(slope*length_sheet_center/max_length_sheet_center),0,n_metal_sheet_flat);linspace(slope*length_sheet_center/max_length_sheet_center,0,n_metal_sheet_flat)],'edgecolor','none');

colormap(cMap);
caxis([0,0.2])

plot([2*pi-length_sheet_center,2*pi,2*pi,2*pi-length_sheet_center,2*pi-length_sheet_center],...
    [y_metal_sheet_end+metal_sheet_width,metal_sheet_width,-metal_sheet_width,y_metal_sheet_end-metal_sheet_width,y_metal_sheet_end+metal_sheet_width],'-k','LineWidth',1);

surf(x_center+[cos(theta_sheet_center);cos(theta_sheet_center)],y_center+[metal_sheet_width*ones(1,n_metal_sheet);-metal_sheet_width*ones(1,n_metal_sheet)],[1+sin(theta_sheet_center);1+sin(theta_sheet_center)],'edgecolor',sheet_color,'facecolor',sheet_color);

plot_wheel(x_center,y_center,omega_center*time)

%campos([-86.3072   -0.5041   19.3854])
%camtarget([2.5000         0    1.0000])
%amva( 6.3648)

%camproj perspective
axis equal
ylim([-2,4]);
end

function plot_wheel(xc,yc,theta)
global text_color
n_wheel = 40;
n_wheel_markers = 15;
theta_wheel = linspace(theta,2*pi+theta,n_wheel);
theta_wheel_markers = linspace(theta,2*pi+theta,n_wheel_markers);

wheel_width = 0.45;

wheel_color = [65,64,69]/255;
surf(xc+[cos(theta_wheel);0.75*cos(theta_wheel)],yc+wheel_width*ones(2,n_wheel),[1+sin(theta_wheel);1+0.75*sin(theta_wheel)],'facecolor',wheel_color,'AlphaDataMapping','none','edgecolor','none');
surf(xc+[cos(theta_wheel);0.75*cos(theta_wheel)],yc-wheel_width*ones(2,n_wheel),[1+sin(theta_wheel);1+0.75*sin(theta_wheel)],'facecolor',wheel_color,'AlphaDataMapping','none','edgecolor','none');
surf(xc+[cos(theta_wheel);cos(theta_wheel)],yc+[wheel_width*ones(1,n_wheel);-wheel_width*ones(1,n_wheel)],[1+sin(theta_wheel);1+sin(theta_wheel)],'facecolor',wheel_color,'AlphaDataMapping','none','edgecolor','none');
surf(xc+[cos(theta_wheel_markers);cos(theta_wheel_markers)],yc+[wheel_width*ones(1,n_wheel_markers);-wheel_width*ones(1,n_wheel_markers)],[1+sin(theta_wheel_markers);1+sin(theta_wheel_markers)],'facecolor','none','AlphaDataMapping','none');
surf(xc+0.75*[cos(theta_wheel);cos(theta_wheel)],yc+[wheel_width*ones(1,n_wheel);-wheel_width*ones(1,n_wheel)],[1+0.75*sin(theta_wheel);1+0.75*sin(theta_wheel)],'facecolor',wheel_color,'AlphaDataMapping','none','edgecolor','none');
plot3(xc+cos(theta_wheel),yc+wheel_width*ones(1,n_wheel),1+sin(theta_wheel),'Color',wheel_color*1.8);
plot3(xc+0.75*cos(theta_wheel),yc+wheel_width*ones(1,n_wheel),1+0.75*sin(theta_wheel),'Color',wheel_color*1.8);
plot3(xc+cos(theta_wheel),yc-wheel_width*ones(1,n_wheel),1+sin(theta_wheel),'Color',wheel_color*1.8);
plot3(xc+0.75*cos(theta_wheel),yc-wheel_width*ones(1,n_wheel),1+0.75*sin(theta_wheel),'Color',wheel_color*1.8);
end