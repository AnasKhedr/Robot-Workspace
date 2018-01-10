%% Values
% Lmin = input('Lmin = ');
% Lmax = input('Lmax = ');
% LStep= input('LStep = ');
% angmin = input('angMin = ');
% angmax = input('angMax = ');
% angStep = input('angStep = ');

clear all
Lmin   = 1;
Lmax   = 3;
LStep  = 0.01;
angmin = 10;
angmax = 20; 
angStep= 0.01;

%% Method 1
%{
figure, hold on, grid on;
tic
for L = Lmin:LStep:Lmax
    for ang = angmin:angStep:angmax         %angle in degrees
        x = L*cos(ang*pi/180);              %convert angle from degree to radian
        y = L*sin(ang*pi/180);
        plot(x,y,'*');
    end
end
disp('Method 1:');
toc
%}
%% Method 2
%{
tic
x=0; y=0;
for L = Lmin:LStep:Lmax
    for ang = angmin:angStep:angmax         %angle in degrees
        x = [x L*cos(ang*pi/180)];          %convert angle from degree to radian
        y = [y L*sin(ang*pi/180)];
    end
end
plot(x,y,'*');
disp('Method 2:');
toc
%}
%% Method 3
%{
tic
x=[]; y=[]; count = 0;
for L = Lmin:LStep:Lmax
    for ang = angmin:angStep:angmax          %angle in degrees
        count = count + 1;
        x(count) = L*cos(ang*pi/180);        %convert angle from degree to radian
        y(count) = L*sin(ang*pi/180);
    end
end
plot(x,y,'*');
disp('Method 3:');
toc
%}
%% Method 4
%{
tic
x=zeros(1,10000); y=x; count = 0;
for L = Lmin:LStep:Lmax
    for ang = angmin:angStep:angmax          %angle in degrees
        count = count + 1;
        x(count) = L*cos(ang*pi/180);        %convert angle from degree to radian
        y(count) = L*sin(ang*pi/180);
    end
end
plot(x,y,'*');
disp('Method 4:');
toc
%}
%% Method 5
%%{
tic
size_X = ((Lmax-Lmin)/LStep) + 1;
size_Y = ((angmax-angmin)/angStep) +1;
size = size_X * size_Y;
x=zeros(1,size); y=x; count = 0;
for L = Lmin:LStep:Lmax
    for ang = angmin:angStep:angmax          %angle in degrees
        count = count + 1;
        x(count) = L*cos(ang*pi/180);        %convert angle from degree to radian
        y(count) = L*sin(ang*pi/180);
    end
end
plot(x,y,'*');
disp('Method 5:');
toc
%%}