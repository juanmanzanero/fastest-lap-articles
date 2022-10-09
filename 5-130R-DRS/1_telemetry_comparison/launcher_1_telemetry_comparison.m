close all
if ismac
    font_name = 'Formula1';
else
    font_name = 'Formula1 Display Regular';
end
set(groot,'defaultAxesFontName',font_name);

load('data.mat');

background_color =  [13/255, 17/255, 23/255];
text_color = [201,209,217]/255;
blue   = [0   0.447000000000000   0.741];

h = figure('Position', [196   471   975   226],'Color',background_color);
plot(s_ref+40,speed_ref,'LineWidth',2)
hold on
plot(s,u,'LineWidth',2)
xlim([2500,5800]);

title('Simulation - last section of Suzuka','Color',text_color)
set(gca,'FontSize',20);

ax = h.CurrentAxes;
ylim([50,325])
yticks(50:25:325);
xticks([2500:125:5800])

ax.YTickLabel = cellfun(@(f)([f,'km/h']),ax.YTickLabel,'UniformOutput',false);
ax.XTickLabel = cellfun(@(f)([f,'m']),ax.XTickLabel,'UniformOutput',false);

ax.YTickLabel(2:2:end) = {''};
ax.XTickLabel(2:4:end) = {''};
ax.XTickLabel(3:4:end) = {''};
ax.XTickLabel(4:4:end) = {''};
ax.Color = background_color;
ax.XColor = text_color;
ax.YColor = text_color;
legend({'Telemetry (fastf1)','Simulation'},'AutoUpdate','off','Location','best','TextColor',text_color,'Color',background_color)
patch([4900,5100,5100,4900],[60,60,315,315],[1,0,0],'FaceAlpha',0.2,'LineStyle','none');
text(5000,260,'130R','Color',text_color,'horizontalAlignment','center','FontName',font_name,'FontSize',20)
grid on
box on
set(h, 'InvertHardcopy', 'off')
print(h,'telemetry_comparison.png','-dpng','-r300')
