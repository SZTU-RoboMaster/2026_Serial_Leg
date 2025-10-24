%计算不同腿长下适合的K矩阵，再进行多项式拟合，得到2*6矩阵每个参数对应的多项式参数
tic
j=1;
leg=0.1:0.01:0.4; %100mm ~10mm ~ 400mm
for i=leg
    k=get_k_length(i);  % lqr函数返回一个 p x n 的矩阵
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

% x0=leg;              %步长为0.1
% y11=polyval(a11,x0);          %返回值y0是对应于x0的函数值
% y12=polyval(a12,x0);          %返回值y0是对应于x0的函数值
% y13=polyval(a13,x0);          %返回值y0是对应于x0的函数值
% y14=polyval(a14,x0);          %返回值y0是对应于x0的函数值
% y15=polyval(a15,x0);          %返回值y0是对应于x0的函数值
% y16=polyval(a16,x0);          %返回值y0是对应于x0的函数值
% 
% y21=polyval(a21,x0);          %返回值y0是对应于x0的函数值
% y22=polyval(a22,x0);          %返回值y0是对应于x0的函数值
% y23=polyval(a23,x0);          %返回值y0是对应于x0的函数值
% y24=polyval(a24,x0);          %返回值y0是对应于x0的函数值
% y25=polyval(a25,x0);          %返回值y0是对应于x0的函数值
% y26=polyval(a26,x0);          %返回值y0是对应于x0的函数值
% subplot(3,4,1);plot(leg,k11,'o',x0,y11,'r');xlabel('x');ylabel('y');title('k11');
% subplot(3,4,2);plot(leg,k12,'o',x0,y12,'r');xlabel('x');ylabel('y');title('k12');
% subplot(3,4,5);plot(leg,k13,'o',x0,y13,'r');xlabel('x');ylabel('y');title('k13');
% subplot(3,4,6);plot(leg,k14,'o',x0,y14,'r');xlabel('x');ylabel('y');title('k14');
% subplot(3,4,9);plot(leg,k15,'o',x0,y15,'r');xlabel('x');ylabel('y');title('k15');
% subplot(3,4,10);plot(leg,k16,'o',x0,y16,'r');xlabel('x');ylabel('y');title('k16');
% 
% subplot(3,4,3);plot(leg,k21,'o',x0,y21,'r');xlabel('x');ylabel('y');title('k21');
% subplot(3,4,4);plot(leg,k22,'o',x0,y22,'r');xlabel('x');ylabel('y');title('k22');
% subplot(3,4,7);plot(leg,k23,'o',x0,y23,'r');xlabel('x');ylabel('y');title('k23');
% subplot(3,4,8);plot(leg,k24,'o',x0,y24,'r');xlabel('x');ylabel('y');title('k24');
% subplot(3,4,11);plot(leg,k25,'o',x0,y25,'r');xlabel('x');ylabel('y');title('k25');
% subplot(3,4,12);plot(leg,k26,'o',x0,y26,'r');xlabel('x');ylabel('y');title('k26');
% grid on;                   %添加网格线
% set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);  %将网格线变成虚线
% fprintf('fp32 a11[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a11(1),a11(2),a11(3),a11(4));
% fprintf('fp32 a12[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a12(1),a12(2),a12(3),a12(4));
% fprintf('fp32 a13[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a13(1),a13(2),a13(3),a13(4));
% fprintf('fp32 a14[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a14(1),a14(2),a14(3),a14(4));
% fprintf('fp32 a15[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a15(1),a15(2),a15(3),a15(4));
% fprintf('fp32 a16[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a16(1),a16(2),a16(3),a16(4));
% 
% fprintf('fp32 a21[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a21(1),a21(2),a21(3),a21(4));
% fprintf('fp32 a22[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a22(1),a22(2),a22(3),a22(4));
% fprintf('fp32 a23[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a23(1),a23(2),a23(3),a23(4));
% fprintf('fp32 a24[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a24(1),a24(2),a24(3),a24(4));
% fprintf('fp32 a25[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a25(1),a25(2),a25(3),a25(4));
% fprintf('fp32 a26[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a26(1),a26(2),a26(3),a26(4));
toc
