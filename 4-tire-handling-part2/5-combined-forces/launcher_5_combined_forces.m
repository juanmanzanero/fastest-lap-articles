clear all
clc
close all

set(groot,'defaultAxesFontName','Formula1');

background_color =  [1,1,1];
text_color = [13/255, 17/255, 23/255];
blue   = [0   0.447000000000000   0.741];
orange = [0.850000000000000   0.325000000000000   0.098000000000000];

tire_data = readstruct('./limebeer-2014-f1.xml').front_tire;

Fz = 3000; 

lambda = deg2rad(linspace(-15,15,1000));

kappa = (0.14:-0.02:0.0)/0.11;

h = figure('Color',background_color);
hold on
colors = jet(numel(kappa));
for i = 1 : numel(kappa)
   plot(rad2deg(lambda), pacejka_model(Fz,kappa(i),lambda,tire_data),'Color',colors(i,:),'LineWidth',3);
end

ylim([-6000,6000]);

h.CurrentAxes.Color = background_color;
h.CurrentAxes.XAxis.Color = text_color;
h.CurrentAxes.YAxis.Color = text_color;
box on
grid on
ytickformat('%d')
h.CurrentAxes.YAxis.Exponent = 0;


h.CurrentAxes.XTick = h.CurrentAxes.XTick;
h.CurrentAxes.YTick = h.CurrentAxes.YTick;

for i = 1 : numel(h.CurrentAxes.YTick)
    h.CurrentAxes.YTickLabel{i} = [h.CurrentAxes.YTickLabel{i},'N'];
end

h.CurrentAxes.FontSize = 14;
h.CurrentAxes.XMinorTick = 'on';
h.CurrentAxes.YMinorTick = 'on';
h.CurrentAxes.LineWidth = 2;
xlabel('lateral slip, lambda [deg]','FontSize',20);
ylabel('lateral force, Fy [N]','FontSize',20)
% Add a legend
kappa_string = cell(1,numel(kappa));

for i = 1:numel(kappa)
   kappa_string{i} = [num2str(kappa(i)*100*0.11) , '%'];
end

lgnd = legend(kappa_string,'Location','Southeast','Color',background_color,'TextColor',text_color,'EdgeColor','none','FontSize',14);
lgnd.Position(1) = lgnd.Position(1) - 0.1;
lgnd.Position(3) = 1.5*lgnd.Position(3);
x_annotation = [lgnd.Position(1),lgnd.Position(2)+lgnd.Position(4),lgnd.Position(3),0.1];

annotation('textbox',x_annotation,'String','longitudinal slip','FaceAlpha',0.0,'Color',text_color,'FontSize',14,'FontName','Formula1','HorizontalAlignment','center','VerticalAlignment','Bottom','LineStyle','none');
annotation('line',[lgnd.Position(1),lgnd.Position(1)+lgnd.Position(3)],0.01+[lgnd.Position(2)+lgnd.Position(4),lgnd.Position(2)+lgnd.Position(4)],'Color',text_color,'LineWidth',1)

arrow = annotation('arrow',[0.0+15,8+15]/30,[3000+6000,7500]/12000,'LineWidth',2);

set(h, 'InvertHardcopy', 'off');
savefig(h,'combined_lateral_force');
print(h,'combined_lateral_force','-dpng','-r300');


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