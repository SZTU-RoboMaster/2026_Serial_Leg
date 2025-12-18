%计算不同腿长下适合的K矩阵，再进行多项式拟合，得到2*6矩阵每个参数对应的多项式参数
tic
j=1;
leg=0.1:0.01:0.4; % 100mm ~10mm ~ 400mm

dt = 0.005;

for i=leg
    k=get_k_length(i,dt);  % lqr函数返回一个 p x n 的矩阵
    k11(j) = k(1,1);    % T: theta的增益
    k12(j) = k(1,2);    % T: theta_dot的增益
    k13(j) = k(1,3);    % T: x的增益
    k14(j) = k(1,4);    % T: x_dot的增益
    k15(j) = k(1,5);    % T: phi的增益
    k16(j) = k(1,6);    % T: phi_dot的增益

    k21(j) = k(2,1);
    k22(j) = k(2,2);
    k23(j) = k(2,3);
    k24(j) = k(2,4);
    k25(j) = k(2,5);
    k26(j) = k(2,6);
    j=j+1;
end

a11=polyfit(leg,k11,3); % a11(1)*x^3 + a11(2)x^2 + a11(3)x^1 + a11(4) a11返回4个值
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

% fprintf('float wheel_K[6] = {\n');
% fprintf('%ff,%ff,%ff,%ff,%ff,%ff\n',k(1,1),k(1,2),k(1,3),k(1,4),k(1,5),k(1,6));
% fprintf('};');
% fprintf('float joint_K[6] = {\n');
% fprintf('%ff,%ff,%ff,%ff,%ff,%ff\n',k(2,1),k(2,2),k(2,3),k(2,4),k(2,5),k(2,6));
% fprintf('};');

fprintf('float wheel_fitting_factor[6][4] = {\n');
fprintf('{%ff,%ff,%ff,%ff},\n',a11(1,1),a11(1,2),a11(1,3),a11(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n\n',a12(1,1),a12(1,2),a12(1,3),a12(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n',a13(1,1),a13(1,2),a13(1,3),a13(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n\n',a14(1,1),a14(1,2),a14(1,3),a14(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n',a15(1,1),a15(1,2),a15(1,3),a15(1,4));
fprintf('{%ff,%ff,%ff,%ff}\n',a16(1,1),a16(1,2),a16(1,3),a16(1,4));
fprintf('};');

fprintf('float joint_fitting_factor[6][4] = {\n');
fprintf('{%ff,%ff,%ff,%ff},\n',a21(1,1),a21(1,2),a21(1,3),a21(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n\n',a22(1,1),a22(1,2),a22(1,3),a22(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n',a23(1,1),a23(1,2),a23(1,3),a23(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n\n',a24(1,1),a24(1,2),a24(1,3),a24(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n',a25(1,1),a25(1,2),a25(1,3),a25(1,4));
fprintf('{%ff,%ff,%ff,%ff}\n',a26(1,1),a26(1,2),a26(1,3),a26(1,4));
fprintf('};');

toc
