%% Values
% Lmin = input('Lmin = ');
% Lmax = input('Lmax = ');
% LStep= input('LStep = ');
% angmin_1 = input('angMin 1 = ');
% angmax_1 = input('angMax 1 = ');
% angmin12 = input('angMin 2 = ');
% angmax2 = input('angMax 2 = ');
% angStep = input('angStep = ');

clear all
Lmin     = 1;
Lmax     = 2;
LStep    = 0.05;
angmin_1 = 10;
angmax_1 = 20;
angmin_2 = 10;
angmax_2 = 20;
angStep= 0.1;

%% Method 1
%{
figure, hold on, grid on;
tic
for L = Lmin:LStep:Lmax
    for ang1 = angmin_1:angStep:angmax_1         %angle in degrees
        for ang2 = angmin_2:angStep:angmax_2
            x = L*sin(ang1*pi/180)*cos(ang2*pi/180);        %convert angle from degree to radian
            y = L*sin(ang1*pi/180)*sin(ang2*pi/180);
            z = L*cos(ang1*pi/180);
            plot3(x,y,z,'*');
        end
    end
end
disp('Method 1:');
toc
%}
%% Method 2
%{
tic
x=0; y=0; z=0;
for L = Lmin:LStep:Lmax
    for ang1 = angmin_1:angStep:angmax_1         %angle in degrees
        for ang2 = angmin_2:angStep:angmax_2
            x = [x L*sin(ang1*pi/180)*cos(ang2*pi/180)];        %convert angle from degree to radian
            y = [y L*sin(ang1*pi/180)*sin(ang2*pi/180)];
            z = [z L*cos(ang1*pi/180)];
        end
    end
end
plot3(x,y,z,'*');
disp('Method 2:');
toc
%}
%% Method 3
%{
tic
x=[]; y=[]; z=[]; count = 0;
for L = Lmin:LStep:Lmax
    for ang1 = angmin_1:angStep:angmax_1          %angle in degrees
        for ang2 = angmin_2:angStep:angmax_2
            count = count + 1;
            x(count) = L*sin(ang1*pi/180)*cos(ang2*pi/180);        %convert angle from degree to radian
            y(count) = L*sin(ang1*pi/180)*sin(ang2*pi/180);
            z(count) = L*cos(ang1*pi/180);
        end
    end
end
plot3(x,y,z,'*');
disp('Method 3:');
toc
%}
%% Method 4
%{
tic
x=zeros(1,200); y=x; z=x; count = 0;
for L = Lmin:LStep:Lmax
    for ang1 = angmin_1:angStep:angmax_1          %angle in degrees
        for ang2 = angmin_2:angStep:angmax_2
            count = count + 1;
            x(count) = L*sin(ang1*pi/180)*cos(ang2*pi/180);        %convert angle from degree to radian
            y(count) = L*sin(ang1*pi/180)*sin(ang2*pi/180);
            z(count) = L*cos(ang1*pi/180);
        end
    end
end
plot3(x,y,z,'*');
disp('Method 4:');
toc
%}
%% Method 5
%%{
tic
size_X = ((Lmax-Lmin)/LStep) + 1;
size_Y = ((angmax_1-angmin_1)/angStep) +1;
size_Z = ((angmax_2-angmin_2)/angStep) +1;
size = size_X * size_Y * size_Z;
x=zeros(1,size); y=x; z=x; count = 0;
for L = Lmin:LStep:Lmax
    for ang1 = angmin_1:angStep:angmax_1          %angle in degrees
        for ang2 = angmin_2:angStep:angmax_2
            count = count + 1;
            x(count) = L*sin(ang1*pi/180)*cos(ang2*pi/180);        %convert angle from degree to radian
            y(count) = L*sin(ang1*pi/180)*sin(ang2*pi/180);
            z(count) = L*cos(ang1*pi/180);
        end
    end
end
plot3(x,y,z,'*');
disp('Method 5:');
toc
%%}