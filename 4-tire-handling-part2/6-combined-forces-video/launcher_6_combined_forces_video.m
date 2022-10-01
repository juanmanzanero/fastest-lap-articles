clear all
clc
close all

global orange
set(groot,'defaultAxesFontName','Formula1');

frame_rate = 15;
dtime = 1/frame_rate;

% Configuration
video_type = 'max-lat';

if strcmpi(video_type, 'max-lon')
    time = 0.0:dtime:5;
    kappa_value = repmat(0.11*0.7512,1,numel(time));
    lambda_value = repmat(0.0,1,numel(time));    
elseif strcmpi(video_type, 'max-lat')
    time = 0.0:dtime:2;
    kappa_value = repmat(0.0,1,numel(time));
    lambda_value = repmat(deg2rad(9)*0.7512,1,numel(time));
    
elseif strcmpi(video_type, 'lat-plus-long')
    
    time = 0.0:dtime:5;
    kappa_value = repmat(0.0,1,numel(time));
    kappa_value(31:60) = linspace(0,0.11*0.7512,30);
    kappa_value(61:end) = 0.11*0.7512;
    lambda_value = repmat(deg2rad(9)*0.7512,1,numel(time));
    
elseif strcmpi(video_type, 'lat-to-long')
    
    time = 0.0:dtime:5;
    kappa_value = repmat(0.0,1,numel(time));
    kappa_value(31:60) = 0.11*0.7512*sin(linspace(0,pi/2,30));
    kappa_value(61:end) = 0.11*0.7512;
    lambda_value = repmat(deg2rad(9)*0.7512,1,numel(time));    
    lambda_value(31:60) = deg2rad(9)*0.7512*cos(linspace(0,pi/2,30));
    lambda_value(61:end) = 0.0;
else
    error();
end


assert(numel(time) == numel(kappa_value));
assert(numel(time) == numel(lambda_value));

background_color =  [1,1,1];
text_color = [13/255, 17/255, 23/255];
blue   = [0   0.447000000000000   0.741];
orange = [0.850000000000000   0.325000000000000   0.098000000000000];

tire_data = readstruct('./limebeer-2014-f1.xml').front_tire;

Fz = 3000;

lambda = deg2rad(linspace(0,15,1001));
kappa = linspace(0,0.15/0.11,1000);

h = figure;
h.Color = background_color;
h.Position = get(0,'screensize');
cmap = jet;

[Kappa,Lambda] = meshgrid(kappa,lambda);
[Fy,Fx] = pacejka_model(Fz,Kappa,Lambda,tire_data);
ax_fx = axes('Position',[0.08,0.1,0.4,0.6]);
hold on
contourf(Kappa*0.11*100,rad2deg(Lambda),Fx,30);
colormap(cmap);


h_target_fx(1) = plot(kappa_value(1)*100, rad2deg(lambda_value(1)), 'or','MarkerSize',8,'LineWidth',20);
h_target_fx(2) = plot(kappa_value(1)*100, rad2deg(lambda_value(1)), 'ow','MarkerSize',16,'LineWidth',20);
h_target_fx(3) = plot(kappa_value(1)*100, rad2deg(lambda_value(1)), 'or','MarkerSize',24,'LineWidth',20);
h_target_fx(4) = plot(kappa_value(1)*100, rad2deg(lambda_value(1)), 'ow','MarkerSize',32,'LineWidth',20);
h_target_fx(5) = plot(kappa_value(1)*100, rad2deg(lambda_value(1)), 'or','MarkerSize',40,'LineWidth',20);

annotation('textbox',[0.1,0.6,0.2,0.07],'horizontalalignment','center','verticalalignment','middle','String','longitudinal force, Fx','FontName','Formula1','FontSize',20,'BackgroundColor',background_color)
ylabel('lateral slip, lambda [deg]')
xlabel('longitudinal slip, kappa [%]')
ax_fx.FontSize = 20;


ax_fy = axes('Position',[0.55,0.1,0.4,0.6]);
hold all
contourf(Kappa*0.11*100,rad2deg(Lambda),Fy,30);
CLim = [0,max(max(Fy))]*1.05;
ax_fx.CLim = CLim;
ax_fy.CLim = CLim;

assert(CLim(1) == 0);
f_color = @(F)(interp1(linspace(0,1,size(cmap,1)),cmap,F/CLim(2)));
h_target_fy(1) = plot(kappa_value(1)*100, rad2deg(lambda_value(1)), 'or','MarkerSize',8,'LineWidth',20);
h_target_fy(2) = plot(kappa_value(1)*100, rad2deg(lambda_value(1)), 'ow','MarkerSize',16,'LineWidth',20);
h_target_fy(3) = plot(kappa_value(1)*100, rad2deg(lambda_value(1)), 'or','MarkerSize',24,'LineWidth',20);
h_target_fy(4) = plot(kappa_value(1)*100, rad2deg(lambda_value(1)), 'ow','MarkerSize',32,'LineWidth',20);
h_target_fy(5) = plot(kappa_value(1)*100, rad2deg(lambda_value(1)), 'or','MarkerSize',40,'LineWidth',20);

xlabel('longitudinal slip, kappa [%]')
ax_fy.FontSize = 20;
annotation('textbox',[0.73,0.6,0.2,0.07],'horizontalalignment','center','verticalalignment','middle','String','lateral force, Fy','FontName','Formula1','FontSize',20,'BackgroundColor',background_color)



ax_tire = axes('Position',[0.3,0.6,0.4,0.38]);
hold all
n_threads = 10;
dx = 2*pi/n_threads;

x_grid = -3:dx:3;
x_grid_length = x_grid(end) - x_grid(1);

y_grid = -1.5:dx:1.5;
y_grid_length = y_grid(end) - y_grid(1);

theta_grid = linspace(0,2*pi,n_threads+1);
theta_grid = theta_grid(1:end-1);

for i_grid = 1 : numel(x_grid)
    h_x_grid(i_grid) = plot([x_grid(i_grid), x_grid(i_grid)], [-1.5,1.5], '-k');
end

for i_grid = 1 : numel(y_grid)
    h_y_grid(i_grid) = plot([-3,3], [y_grid(i_grid), y_grid(i_grid)], '-k');
end




plot(polybuffer( polyshape([-0.75 -0.75 0.75 0.75], [-0.3 0.3 0.3 -0.3]),0.25),'FaceColor',[13/255, 17/255, 23/255]*4,'FaceAlpha',1.0)

[Fy_values,Fx_values] = pacejka_model(Fz,kappa_value/0.11,lambda_value,tire_data);
h_arrow1 = draw_arrow([0,Fx_values(1)]/5000,[0,0]/5000,f_color(Fx_values(1)));
h_arrow2 = draw_arrow([0,0]/5000,[0,-Fy_values(1)]/5000,f_color(Fy_values(1)));
h_arrow3 = draw_arrow([0,Fx_values(1)]/5000,[0,-Fy_values(1)]/5000,f_color(norm([Fx_values(1),Fy_values(1)])));


for i_theta = 1 : numel(theta_grid)
    h_theta(i_theta) = plot([cos(theta_grid(i_theta)),cos(theta_grid(i_theta))],[-0.3,0.3],'y','LineWidth',2);
    h_theta_dash(i_theta) = plot([cos(theta_grid(i_theta)),cos(theta_grid(i_theta))],[-0.3,0.3],'--y','LineWidth',2);
    
    if ( wrapTo2Pi(theta_grid(i_theta)) <= pi )
        h_theta(i_theta).Visible = 'on'
        h_theta_dash(i_theta).Visible = 'on';
    else
        h_theta(i_theta).Visible = 'off';
        h_theta_dash(i_theta).Visible = 'on';
    end
    
end

axis equal
xlim([-3,3]);
ax_tire.YLimMode = 'manual';
ax_tire.XTick = [];
ax_tire.YTick = [];
box on



omega_value = (kappa_value + 1);
theta_dash_grid = theta_grid;
drawnow;
for i_time = 1 : numel(time)
    % Update xgrid positions
    x_grid = x_grid - dtime;
    x_grid(x_grid < -3) = x_grid(x_grid < -3) + x_grid_length + dx;
    
    for i_x_grid = 1 : numel(x_grid)
        h_x_grid(i_x_grid).XData = [x_grid(i_x_grid),x_grid(i_x_grid)];
    end
    
    % Update ygrid positions
    y_grid = y_grid - lambda_value(i_time)*dtime;
    y_grid(y_grid < -1.5) = y_grid(y_grid < -1.5) + y_grid_length + dx;
    
    
    for i_y_grid = 1 : numel(y_grid)
        h_y_grid(i_y_grid).YData = [y_grid(i_y_grid),y_grid(i_y_grid)];
    end
    
    % Update radii
    theta_grid = theta_grid - omega_value(i_time)*dtime;
    theta_dash_grid = theta_dash_grid - dtime;
    
    for i_theta = 1 : numel(theta_grid)
        h_theta(i_theta).XData = [cos(theta_grid(i_theta)),cos(theta_grid(i_theta))];
        h_theta_dash(i_theta).XData = [cos(theta_dash_grid(i_theta)),cos(theta_dash_grid(i_theta))];
        
        if ( wrapTo2Pi(theta_grid(i_theta)) <= pi )
            h_theta(i_theta).Visible = 'on';
        else
            h_theta(i_theta).Visible = 'off';
        end
        
        if ( wrapTo2Pi(theta_dash_grid(i_theta)) <= pi )
            h_theta_dash(i_theta).Visible = 'on';
        else
            h_theta_dash(i_theta).Visible = 'off';
        end        
    end
    
    for i_target = 1 : numel(h_target_fx)
        h_target_fx(i_target).XData = kappa_value(i_time)*100;
        h_target_fx(i_target).YData = rad2deg(lambda_value(i_time));
    end
    for i_target = 1 : numel(h_target_fy)
        h_target_fy(i_target).XData = kappa_value(i_time)*100;
        h_target_fy(i_target).YData = rad2deg(lambda_value(i_time));
    end
    
    
    delete(h_arrow1);
    delete(h_arrow2);
    delete(h_arrow3);
    h_arrow1 = draw_arrow([0,Fx_values(i_time)]/5000,[0,0]/5000,f_color(Fx_values(i_time)));
    h_arrow2 = draw_arrow([0,0]/5000,[0,-Fy_values(i_time)]/5000,f_color(Fy_values(i_time)));
    h_arrow3 = draw_arrow([0,Fx_values(i_time)]/5000,[0,-Fy_values(i_time)]/5000,f_color(norm([Fx_values(i_time),Fy_values(i_time)])));
    pause(dtime)
end





function [Fy,Fx,rho,lambda_max_real] = pacejka_model(Fz,kappa,lambda,tire_data)
Fz_1 = tire_data.reference_load_1.Text;
Fz_2 = tire_data.reference_load_2.Text;
mu_x_1 = tire_data.mu_x_max_1;
mu_x_2 = tire_data.mu_x_max_2;
kappa_max_1 = tire_data.kappa_max_1;
kappa_max_2 = tire_data.kappa_max_2;
mu_y_1 = tire_data.mu_y_max_1;
mu_y_2 = tire_data.mu_y_max_2;
lambda_max_1 = deg2rad(tire_data.lambda_max_1.Text);
lambda_max_2 = deg2rad(tire_data.lambda_max_2.Text);

Qx = tire_data.Qx;
Qy = tire_data.Qy;
Sx = pi/(2*atan(Qx));
Sy = pi/(2*atan(Qy));
mu_x_max = (Fz-Fz_1)*(mu_x_2-mu_x_1)/(Fz_2-Fz_1) + mu_x_1;
mu_y_max = (Fz-Fz_1)*(mu_y_2-mu_y_1)/(Fz_2-Fz_1) + mu_y_1;
kappa_max = (Fz-Fz_1)*(kappa_max_2-kappa_max_1)/(Fz_2-Fz_1) + kappa_max_1;
lambda_max = (Fz-Fz_1)*(lambda_max_2-lambda_max_1)/(Fz_2-Fz_1) + lambda_max_1;
kappa = kappa.*kappa_max;
rho = sqrt((kappa./kappa_max).^2 + (lambda./lambda_max).^2);
Fx = mu_x_max.*Fz.*sin(Qx.*atan(Sx*rho)).*kappa./(kappa_max.*rho);
Fy = mu_y_max.*Fz.*sin(Qy.*atan(Sy*rho)).*lambda./(lambda_max.*rho);

lambda_max_real = lambda_max*0.751247350722835;
end


function handles = draw_arrow(x,y,color)

head_length = 0.2;
handles(1) = plot([x(1),x(1) + (1-head_length)*(x(2)-x(1))],[y(1),y(1) + (1-head_length)*(y(2)-y(1))],'-','LineWidth',4,'Color',color);


triangle_base = [x(1),y(1)] + (1-head_length)*[x(2)-x(1),y(2)-y(1)];
dr = [x(2)-x(1),y(2)-y(1)];

triangle_P0 = triangle_base + [dr(2),-dr(1)]*head_length*0.4;
triangle_P1 = triangle_base - [dr(2),-dr(1)]*head_length*0.4;

handles(2) = patch([x(2),triangle_P0(1),triangle_P1(1)], [y(2), triangle_P0(2), triangle_P1(2)],color,'EdgeColor',color);
end