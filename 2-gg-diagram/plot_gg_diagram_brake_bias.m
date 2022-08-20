
n_frames = 5*30;
brake_bias_analysis_ay = reshape([brake_bias_analysis.ay],numel(brake_bias_analysis(1).ay),numel(brake_bias_analysis));
brake_bias_analysis_ax_max = reshape([brake_bias_analysis.ax_max],numel(brake_bias_analysis(1).ax_max),numel(brake_bias_analysis));
brake_bias_analysis_ax_min = reshape([brake_bias_analysis.ax_min],numel(brake_bias_analysis(1).ax_min),numel(brake_bias_analysis));

brake_bias_analysis_ay = interp1(brake_biases, brake_bias_analysis_ay', linspace(brake_biases(1),brake_biases(end),n_frames));
brake_bias_analysis_ax_max = interp1(brake_biases, brake_bias_analysis_ax_max', linspace(brake_biases(1),brake_biases(end),n_frames));
brake_bias_analysis_ax_min = interp1(brake_biases, brake_bias_analysis_ax_min', linspace(brake_biases(1),brake_biases(end),n_frames));

background_color =  [13/255, 17/255, 23/255];
text_color = [201,209,217]/255;

h = figure;
h.Color = background_color;
plot(linspace(brake_biases(1),brake_biases(end),n_frames)*100, brake_bias_analysis_ax_min(:,1)/9.81,'LineWidth',3,'Color',text_color);
set(gca,'MinorGridLineStyle','-')
ax = h.CurrentAxes;
ax.FontSize = 20;
ylim([-3.5,-1.5])
ax.Color = background_color;
ax.GridColor = text_color;
ax.XColor = text_color;
ax.YColor = text_color;
xlabel('brake-bias [%]')
ylabel('maximum deceleration [g]')
grid minor
set(h, 'InvertHardcopy', 'off');
print(h,'brake_bias_ax.png','-dpng','-r300')