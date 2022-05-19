close all;
clear;

%%
exp = {'17','18','19','20','21','22','29','30','31'};

for i = 1:length(exp)
    fig_name = strcat('Results\exp_',exp{i},'_tree.fig');
    fig1 = openfig(fig_name);
    img_name = strcat('Results\exp_',exp{i},'_tree.jpg');
    fig1_lgd = legend('', '', 'Tree 1 Node', 'Tree 1 Edge', 'Tree 2 Node', 'Tree 2 Edge', 'Orientation','horizontal', 'Location','northoutside');
    fig1_lgd.NumColumns = 2;
    saveas(fig1, img_name);
    fig_name = strcat('Results\exp_',exp{i},'_motion_plan.fig');
    fig2 = openfig(fig_name);
    img_name = strcat('Results\exp_',exp{i},'_motion_plan.jpg');
    fig2_lgd = legend('Start Pose', 'Goal Pose', 'Node', 'Edge', 'Orientation','horizontal', 'Location','northoutside');
    fig2_lgd.NumColumns = 2;
    saveas(fig2, img_name);
end