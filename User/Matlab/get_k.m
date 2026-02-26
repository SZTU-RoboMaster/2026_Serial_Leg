
% ============================= 3.1 考虑不同腿长 ============================= %

% 根据机械结构设定最小&最大腿长 此处为100mm~400mm 步长10mm
leg=0.1:0.01:0.4;

% 根据控制频率设置dt 此处为0.001 （与clion中CHASSIS_PERIOD保持一致）
dt = 0.001;

% 遍历每个腿长值
for j = 1:length(leg)

    % 计算当前腿长下的K
    k = get_k_length(leg(j), dt);

    % 提取K第一行元素
    k11(j) = k(1,1); k12(j) = k(1,2); k13(j) = k(1,3);
    k14(j) = k(1,4); k15(j) = k(1,5); k16(j) = k(1,6);

    % 提取K第二行元素
    k21(j) = k(2,1); k22(j) = k(2,2); k23(j) = k(2,3);
    k24(j) = k(2,4); k25(j) = k(2,5); k26(j) = k(2,6);

    % 打印腿长
    % fprintf('leg(%d) = %ff\n', j, leg(j));

end

% ============================= 3.1 多项式拟合 ============================= %

a11=polyfit(leg,k11,3); 
a12=polyfit(leg,k12,3);
a13=polyfit(leg,k13,3);
a14=polyfit(leg,k14,3);
a15=polyfit(leg,k15,3);
a16=polyfit(leg,k16,3);

a21=polyfit(leg,k21,3);
a22=polyfit(leg,k22,3);
a23=polyfit(leg,k23,3);
a24=polyfit(leg,k24,3);
a25=polyfit(leg,k25,3);
a26=polyfit(leg,k26,3);

% ============================= 打印K ============================= %

fprintf('float wheel_fitting_factor[6][4] = \n { \n');
fprintf('{%ff,%ff,%ff,%ff},\n', a11);
fprintf('{%ff,%ff,%ff,%ff},\n\n', a12);
fprintf('{%ff,%ff,%ff,%ff},\n', a13);
fprintf('{%ff,%ff,%ff,%ff},\n\n', a14);
fprintf('{%ff,%ff,%ff,%ff},\n', a15);
fprintf('{%ff,%ff,%ff,%ff}\n', a16);
fprintf('};\n\n');

fprintf('float joint_fitting_factor[6][4] = \n { \n');
fprintf('{%ff,%ff,%ff,%ff},\n', a21);
fprintf('{%ff,%ff,%ff,%ff},\n\n', a22);
fprintf('{%ff,%ff,%ff,%ff},\n', a23);
fprintf('{%ff,%ff,%ff,%ff},\n\n', a24);
fprintf('{%ff,%ff,%ff,%ff},\n', a25);
fprintf('{%ff,%ff,%ff,%ff}\n', a26);
fprintf('};\n\n');

fprintf('运行结束，当前时间：%s\n', datetime("now"));