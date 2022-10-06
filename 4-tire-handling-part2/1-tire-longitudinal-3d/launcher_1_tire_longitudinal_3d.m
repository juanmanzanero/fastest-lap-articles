clear all
clc
close all

global text_color background_color


gif_duration = 4;
frame_rate = 30;
dtime = 1/frame_rate;
filename = 'tire_longitudinal_3d.gif';

%background_color =  [13/255, 17/255, 23/255];
%text_color = [201,209,217]/255;
text_color =  [13/255, 17/255, 23/255];
background_color = [1,1,1];

blue   = [0   0.447000000000000   0.741];
orange = [0.850000000000000   0.325000000000000   0.098000000000000];

t = 0:dtime:10;

for i_time = 1:numel(t)
    fprintf('%d/%d\n',i_time,numel(t))
    
    x = 2*pi-t(i_time);
    plot_frame(x,-4,0.9,x,0,1.0,x,4,1.1,t(i_time))

    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i_time == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
    elseif i_time == numel(t)
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',3);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',dtime);
    end
    close(gcf)
end

function plot_frame(x_left, y_left, omega_left, x_center, y_center, omega_center, x_right, y_right, omega_right, time)
global text_color background_color

h = figure('Visible','off');
h.Color = background_color;
h.Position = [340         269        1226         623];
hold all
h.CurrentAxes.Visible = 'off';

theta_left = omega_left * time;
theta_center = omega_center * time;
theta_right = omega_right * time;

% (1) Plot the grid
x_grid = -5:10;
y_grid = -30:30;

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

theta_sheet_left = linspace(min(max(0.5*pi,theta_left-pi/2),3*pi/2-0.2),3*pi/2-0.2,n_metal_sheet);
theta_sheet_center = linspace(min(max(0.5*pi,theta_center-pi/2),3*pi/2-0.2),3*pi/2-0.2,n_metal_sheet);
theta_sheet_right = linspace(min(max(0.5*pi,theta_right-pi/2),3*pi/2-0.2),3*pi/2-0.2,n_metal_sheet);

max_length_sheet_left = 2*pi/omega_left;
max_length_sheet_center = 2*pi/omega_center;
max_length_sheet_right = 2*pi/omega_right;

length_sheet_left = min(2*pi-x_left, max_length_sheet_left);
length_sheet_center = min(2*pi-x_left, max_length_sheet_center);
length_sheet_right = min(2*pi-x_left, max_length_sheet_right);

cMap = jet;

surf([linspace(2*pi-length_sheet_left,2*pi,n_metal_sheet_flat);linspace(2*pi-length_sheet_left,2*pi,n_metal_sheet_flat)],...
    [y_left + metal_sheet_width*ones(1,n_metal_sheet_flat);y_left-metal_sheet_width*ones(1,n_metal_sheet_flat)],...
    zeros(2,n_metal_sheet_flat),[linspace(-length_sheet_left/max_length_sheet_left,0,n_metal_sheet_flat);linspace(-length_sheet_left/max_length_sheet_left,0,n_metal_sheet_flat)],'edgecolor','none');

surf([linspace(2*pi-length_sheet_center,2*pi,n_metal_sheet_flat);linspace(2*pi-length_sheet_center,2*pi,n_metal_sheet_flat)],...
    [y_center + metal_sheet_width*ones(1,n_metal_sheet_flat);y_center-metal_sheet_width*ones(1,n_metal_sheet_flat)],...
    zeros(2,n_metal_sheet_flat),zeros(2,n_metal_sheet_flat),'edgecolor','none');

surf([linspace(2*pi-length_sheet_right,2*pi,n_metal_sheet_flat);linspace(2*pi-length_sheet_right,2*pi,n_metal_sheet_flat)],...
    [y_right + metal_sheet_width*ones(1,n_metal_sheet_flat);y_right-metal_sheet_width*ones(1,n_metal_sheet_flat)],...
    zeros(2,n_metal_sheet_flat),[linspace(length_sheet_right/max_length_sheet_right,0,n_metal_sheet_flat);linspace(length_sheet_right/max_length_sheet_right,0,n_metal_sheet_flat)],'edgecolor','none');

colormap(cMap);
caxis([-1,1])

plot([2*pi-length_sheet_left,2*pi,2*pi,2*pi-length_sheet_left,2*pi-length_sheet_left],...
    [y_left+metal_sheet_width,y_left+metal_sheet_width,y_left-metal_sheet_width,y_left-metal_sheet_width,y_left+metal_sheet_width],'-k','LineWidth',1);
plot([2*pi-length_sheet_center,2*pi,2*pi,2*pi-length_sheet_center,2*pi-length_sheet_center],...
    [y_center+metal_sheet_width,y_center+metal_sheet_width,y_center-metal_sheet_width,y_center-metal_sheet_width,y_center+metal_sheet_width],'-k','LineWidth',1);
plot([2*pi-length_sheet_right,2*pi,2*pi,2*pi-length_sheet_right,2*pi-length_sheet_right],...
    [y_right+metal_sheet_width,y_right+metal_sheet_width,y_right-metal_sheet_width,y_right-metal_sheet_width,y_right+metal_sheet_width],'-k','LineWidth',1);

surf(x_left+[cos(theta_sheet_left);cos(theta_sheet_left)],[y_left + metal_sheet_width*ones(1,n_metal_sheet);y_left-metal_sheet_width*ones(1,n_metal_sheet)],[1+sin(theta_sheet_left);1+sin(theta_sheet_left)],'edgecolor',sheet_color,'facecolor',sheet_color);
surf(x_center+[cos(theta_sheet_center);cos(theta_sheet_center)],y_center+[metal_sheet_width*ones(1,n_metal_sheet);-metal_sheet_width*ones(1,n_metal_sheet)],[1+sin(theta_sheet_center);1+sin(theta_sheet_center)],'edgecolor',sheet_color,'facecolor',sheet_color);
surf(x_right+[cos(theta_sheet_right);cos(theta_sheet_right)],[y_right + metal_sheet_width*ones(1,n_metal_sheet);y_right-metal_sheet_width*ones(1,n_metal_sheet)],[1+sin(theta_sheet_right);1+sin(theta_sheet_right)],'edgecolor',sheet_color,'facecolor',sheet_color);

plot_wheel(x_left,y_left,omega_left*time)
plot_wheel(x_center,y_center,omega_center*time)
plot_wheel(x_right,y_right,omega_right*time)

campos([-2,0,4])

camproj perspective
axis equal

% Add colorbar
cb = colorbar;
cb.Position = [ 0.8048    0.7127    0.0199    0.2091];
cb.Ticks = [-1,0,1];
cb.TickLabels = {'Extended','Normal','Compressed'};
cb.FontName = 'Courier';
cb.FontWeight = 'bold';
cb.FontSize = 20;

% Add annotations
annotation('textbox',[0.3,0.6,1,0.2],'String','\omega R > v','FitBoxToText','on','LineStyle','none','FontName','Courier','FontSize',20,'horizontalAlignment','left');
annotation('textbox',[0.46,0.6,1,0.2],'String','\omega R = v','FitBoxToText','on','LineStyle','none','FontName','Courier','FontSize',20,'horizontalAlignment','left');
annotation('textbox',[0.62,0.6,1,0.2],'String','\omega R < v','FitBoxToText','on','LineStyle','none','FontName','Courier','FontSize',20,'horizontalAlignment','left');
annotation('textbox',[0.3,0.65,1,0.2],'String','Traction','FitBoxToText','on','LineStyle','none','FontName','Courier','FontSize',20,'horizontalAlignment','left');
annotation('textbox',[0.44,0.65,1,0.2],'String','Free-rolling','FitBoxToText','on','LineStyle','none','FontName','Courier','FontSize',20,'horizontalAlignment','left');
annotation('textbox',[0.62,0.65,1,0.2],'String','Deceleration','FitBoxToText','on','LineStyle','none','FontName','Courier','FontSize',20,'horizontalAlignment','left');
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