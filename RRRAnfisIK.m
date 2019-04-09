%Clear the environment
clear;
clc;
%RRR planar manipulator IK
l1 = 10; % length of first arm
l2 = 7; % length of second arm
l3 = 5; % length of third arm
%these arrays need to be the same size, resolution here affects anfis
%accuracy
theta1 = 0:0.4:pi; % all possible theta1 values
theta2 = 0:0.2:pi/2; % all possible theta2 values
theta3 = -pi/2:0.4:pi/2; % all possible theta3 values

[THETA1,THETA2,THETA3] = meshgrid(theta1,theta2,theta3); % generate a grid of theta1 and theta2 values


%FK calculations
X = (l1 * cos(THETA1)) + (l2 * cos(THETA1 + THETA2)) + (l3 * cos(THETA1 + THETA2 + THETA3));
Y = (l1 * sin(THETA1)) + (l2 * sin(THETA1 + THETA2)) + (l3 * sin(THETA1 + THETA2 + THETA3));
%phi = THETA1 + THETA2 + THETA3; % Where does this get used?

%Plot the generated workspace?
figure(1);
plot(X(:), Y(:),'.');

data1 = [X(:) Y(:) THETA1(:)]; % create x-y-phi-theta1 dataset
data2 = [X(:) Y(:) THETA2(:)]; % create x-y-phi-theta2 dataset
data3 = [X(:) Y(:) THETA3(:)];% create x-y-phi-theta3 dataset


opt = anfisOptions;
opt.InitialFIS = 10;
opt.EpochNumber = 8;
opt.DisplayANFISInformation = 0;
opt.DisplayErrorValues = 0;
opt.DisplayStepSize = 0;
opt.DisplayFinalResults = 0;

%%
% Train anfis1.
disp('--> Training first ANFIS network.')
anfis1 = anfis(data1,opt);


%Train anfis2
disp('--> Training second ANFIS network.')
opt.InitialFIS = 8;
opt.EpochNumber = 8;
anfis2 = anfis(data2,opt);

%train anfis3
disp('--> Training third ANFIS network.')
opt.InitialFIS = 5;
opt.EpochNumber = 8;
anfis3 = anfis(data3,opt);


X = 0:0.25:10; % x coordinates for validation
Y = 10:-0.25:0; % y coordinates for validation, This is just a line currently
PHI = 0 %phi value for validation
figure(2);
plot(X,Y,'-b');
hold on;
%Actual IK calculations, for comparison with anfis


a = Y - (l3*sin(PHI)); % Y of wrist
b = X - (l3*cos(PHI)); % X of wrist

%Get theta2

D = ( ((a.^2)+(b.^2) - l1^2 - l2^2)/(2*l1*l2)); %Costheta2


THETA2D = real(acos(D));

%Get theta1

k1 = l2*sin(THETA2D);
k2 = l1 + l2*cos(THETA2D);


THETA1D = atan2((a.*k2)-(k1.*b),(a.*k1)+(b.*k2));

%get theta 3
THETA3D = PHI - (THETA1D + THETA2D);

%%Plot the calculated IK X&Y positions, this is green
valX = (l1 * cos(THETA1D(:))) + (l2 * cos(THETA1D(:)+ THETA2D(:))) + (l3 * cos(THETA1D(:)+THETA2D(:)+THETA3D(:)));
valY = (l1 * sin(THETA1D(:))) + (l2 * sin(THETA1D(:)+ THETA2D(:))) + (l3 * sin(THETA1D(:)+THETA2D(:)+THETA3D(:)));
plot(valX(:),valY(:),'-g');


XY = [X(:) Y(:) ];
THETA1P = evalfis(XY,anfis1); % theta1 predicted by anfis1
THETA2P = evalfis(XY,anfis2); % theta2 predicted by anfis2
THETA3P = evalfis(XY,anfis3); % theta3 predicted by anfis3


testX = (l1 * cos(THETA1P)) + (l2 * cos(THETA1P+ THETA2P)) + (l3 * cos(THETA1P+THETA2P+THETA3P));
testY = (l1 * sin(THETA1P)) + (l2 * sin(THETA1P+ THETA2P)) + (l3 * sin(THETA1P+THETA2P+THETA3P));
plot(testX,testY,'-r');
hold off;

%error as percentage of the reach radius?

%mean squared error
% theta1diff = (THETA1D(:) - THETA1P).^2; 
% theta2diff = (THETA2D(:) - THETA2P).^2;
% theta3diff = (THETA3D(:) - THETA3P).^2;

%Original metrics ,Is this a good measure of error?
theta1diff = THETA1D(:) - THETA1P; 
theta2diff = THETA2D(:) - THETA2P;
theta3diff = THETA3D(:) - THETA3P;

%Plot the errors for each theta
figure(3);
subplot(3,1,1);
plot(theta1diff);
ylabel('Theta1 Error','fontsize',10)
title('MSE 1','fontsize',10)

subplot(3,1,2);
plot(theta2diff);
ylabel('Theta2 Error','fontsize',10)
title('MSE 2','fontsize',10)

subplot(3,1,3);
plot(theta3diff);
ylabel('Theta3 Error','fontsize',10)
title('MSE 3','fontsize',10)

