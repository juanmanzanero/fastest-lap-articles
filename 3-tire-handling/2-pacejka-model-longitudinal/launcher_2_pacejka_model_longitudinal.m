clear all
clc
close all
%

set(groot,'defaultAxesFontName','Formula1');

background_color =  [13/255, 17/255, 23/255];
text_color = [201,209,217]/255;
blue   = [0   0.447000000000000   0.741];
orange = [0.850000000000000   0.325000000000000   0.098000000000000];

tire_data = readstruct('../limebeer-2014-f1.xml');

Fz = 1000:1000:8000;

kappa = linspace(-0.3,0.3,1000);

h = figure('Color',background_color);
hold on
colors = jet(numel(Fz));
for i = 1 : numel(Fz)
   plot(kappa, pacejka_model_longitudinal(Fz(i),kappa,tire_data),'Color',colors(i,:),'LineWidth',3);
end

Fx_kappa_max_few = zeros(1,numel(Fz));
kappa_max_few = zeros(1,numel(Fz));
for i = 1 : numel(Fz)
   [Fx_kappa_max_few(i),kappa_max_few(i)] =  pacejka_model_longitudinal(Fz(i),kappa_max_few(i),tire_data);
   [Fx_kappa_max_few(i),kappa_max_few(i)] =  pacejka_model_longitudinal(Fz(i),kappa_max_few(i),tire_data);
end

Fz_kappa_max = linspace(500,10000,100);
kappa_max = zeros(1,numel(Fz_kappa_max));
Fx_kappa_max = zeros(1,numel(Fz_kappa_max));

for i = 1 : numel(Fz_kappa_max)
    [Fx_kappa_max(i),kappa_max(i)] = pacejka_model_longitudinal(Fz_kappa_max(i),kappa_max(i),tire_data);
    [Fx_kappa_max(i),kappa_max(i)] = pacejka_model_longitudinal(Fz_kappa_max(i),kappa_max(i),tire_data);
end

plot(kappa_max,Fx_kappa_max,'-','Color',text_color,'LineWidth',0.1)
plot(kappa_max_few, Fx_kappa_max_few,'p','Color',text_color,'MarkerSize',10,'MarkerFaceColor',text_color);
plot(-kappa_max,-Fx_kappa_max,'-','Color',text_color,'LineWidth',0.1)
plot(-kappa_max_few, -Fx_kappa_max_few,'p','Color',text_color,'MarkerSize',10,'MarkerFaceColor',text_color);
ylim([-11000,11000]);

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
xlabel('longitudinal slip, kappa [-]','FontSize',20);
ylabel('longitudinal force, Fx [N]','FontSize',20)
% Add a legend
Fz_string = cell(1,numel(Fz));

for i = 1:numel(Fz)
   Fz_string{i} = [num2str(Fz(i)) , 'N'];
end

lgnd = legend(Fz_string,'Location','Southeast','Color','none','TextColor',text_color,'EdgeColor','none','FontSize',14);

x_annotation = [lgnd.Position(1),lgnd.Position(2)+lgnd.Position(4),lgnd.Position(3),0.1];

annotation('textbox',x_annotation,'String','Fz','FaceAlpha',0.0,'Color',text_color,'FontSize',14,'FontName','Formula1','HorizontalAlignment','center','VerticalAlignment','Bottom','LineStyle','none');
annotation('line',[lgnd.Position(1),lgnd.Position(1)+lgnd.Position(3)],0.01+[lgnd.Position(2)+lgnd.Position(4),lgnd.Position(2)+lgnd.Position(4)],'Color',text_color,'LineWidth',1)


set(h, 'InvertHardcopy', 'off');
savefig(h,'limebeer_longitudinal_force');
print(h,'limebeer_longitudinal_force','-dpng','-r300');


function [Fx,kappa_max_real] = pacejka_model_longitudinal(Fz,kappa,tire_data)
    Fz_1 = tire_data.rear_tire.reference_load_1.Text;
    Fz_2 = tire_data.rear_tire.reference_load_2.Text;
    mu_x_1 = tire_data.rear_tire.mu_x_max_1;
    mu_x_2 = tire_data.rear_tire.mu_x_max_2;
    kappa_max_1 = tire_data.rear_tire.kappa_max_1;
    kappa_max_2 = tire_data.rear_tire.kappa_max_2;
    Qx = tire_data.rear_tire.Qx;
    Sx = pi/(2*atan(Qx));
    mu_x_max = (Fz-Fz_1)*(mu_x_2-mu_x_1)/(Fz_2-Fz_1) + mu_x_1;
    kappa_max = (Fz-Fz_1)*(kappa_max_2-kappa_max_1)/(Fz_2-Fz_1) + kappa_max_1;
    kappa_max_real = kappa_max*0.751247350722835;
    Fx = mu_x_max*Fz.*sin(Qx*atan(Sx*kappa./kappa_max));
end
