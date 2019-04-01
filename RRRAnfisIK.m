%Clear the environment
clear;
clc;
%RRR planar manipulator IK
l1 = 10; % length of first arm
l2 = 7; % length of second arm
l3 = 5; % length of third arm
%these arrays need to be the same size
theta1 = 0:0.1:pi/2; % all possible theta1 values
theta2 = -pi/2:0.2:pi/2; % all possible theta2 values
theta3 = -pi/2:0.2:pi/2; % all possible theta3 values

[THETA1,THETA2,THETA3] = meshgrid(theta1,theta2,theta3); % generate a grid of theta1 and theta2 values


%FK calculations
X = (l1 * cos(THETA1)) + (l2 * cos(THETA1 + THETA2)) + (l3 * cos(THETA1 + THETA2 + THETA3));
Y = (l1 * sin(THETA1)) + (l2 * sin(THETA1 + THETA2)) + (l3 * sin(THETA1 + THETA2 + THETA3));
%phi = THETA1 + THETA2 + THETA3; % Where does this get used?

%What about the wrist location?

data1 = [X(:) Y(:) THETA1(:)]; % create x-y-phi-theta1 dataset
data2 = [X(:) Y(:) THETA2(:)]; % create x-y-phi-theta2 dataset
data3 = [X(:) Y(:) THETA3(:)];% create x-y-phi-theta3 dataset

%%
% The following plot shows all the X-Y data points generated by cycling
% through different combinations of |theta1| and |theta2| and deducing x
% and y coordinates for each. The plot can be generated by using the
% code-snippet shown below. The plot is illustrated further for easier
% understanding.
%
%    plot(X(:),Y(:),'r.'); 
%    axis equal;
%    xlabel('X','fontsize',10)
%    ylabel('Y','fontsize',10)
%    title('X-Y coordinates generated for all theta1 and theta2 combinations using forward kinematics formula','fontsize',10)

opt = anfisOptions;
opt.InitialFIS = 7;
opt.EpochNumber = 8;
opt.DisplayANFISInformation = 0;
opt.DisplayErrorValues = 0;
opt.DisplayStepSize = 0;
opt.DisplayFinalResults = 0;

%%
% Train an ANFIS system using the first set of training data, |data1|.
disp('--> Training first ANFIS network.')
anfis1 = anfis(data1,opt);

%%
% Change the number of input membership functions and train an ANFIS system
% using the second set of training data, |data2|.
disp('--> Training second ANFIS network.')
opt.InitialFIS = 5;
opt.EpochNumber = 8;
anfis2 = anfis(data2,opt);

%train 3rd network here with data3
disp('--> Training third ANFIS network.')
opt.InitialFIS = 5;
opt.EpochNumber = 30;
anfis3 = anfis(data3,opt);


x = 5:1:15; % x coordinates for validation
y = 5:1:15; % y coordinates for validation
PHI = 0 %phi values for validation
%%
% The |theta1| and |theta2| values are deduced mathematically from the x
% and y coordinates using inverse kinematics formulae.
%THIS CURRENTLY ISN'T WORKING

[X,Y] = meshgrid(x,y);

costheta2 = ((X.*X)+(Y.*Y)-(l1*l1)-(l2*l2))/(2*l1*l2)
sintheta2 = sqrt(1 - (costheta2.*costheta2));
THETA2D = atan2(real(sintheta2), real(costheta2));

XW = X - (l3*cos(PHI)); %wrist locations
YW = Y - (l3*sin(PHI));

K1 = l1 + (l2*costheta2);
K2 = (l2*sintheta2);

THETA1D = atan2(real((K1.*YW) - (K2.*XW)),real((K1.*XW)-(K2.*YW)));

THETA3D = PHI - (THETA1D + THETA2D);

XY = [X(:) Y(:) ];
%XYPHI = [X(:) Y(:) PHI(:) ];
THETA1P = evalfis(XY,anfis1); % theta1 predicted by anfis1
THETA2P = evalfis(XY,anfis2); % theta2 predicted by anfis2
THETA3P = evalfis(XY,anfis3); % theta3 predicted by anfis3
%%
% Now, we can see how close the FIS outputs are with respect to the
% deduced values.
%gonna do mean squared error
% theta1diff = (THETA1D(:) - THETA1P).^2; 
% theta2diff = (THETA2D(:) - THETA2P).^2;
% theta3diff = (THETA3D(:) - THETA3P).^2;

%Original metrics ,Is this a good measure of error?
theta1diff = THETA1D(:) - THETA1P; 
theta2diff = THETA2D(:) - THETA2P;
theta3diff = THETA3D(:) - THETA3P;

subplot(3,1,1);
plot(theta1diff);
ylabel('THETA1 Error','fontsize',10)
title('MSE 1','fontsize',10)

subplot(3,1,2);
plot(theta2diff);
ylabel('THETA2 Error','fontsize',10)
title('MSE 2','fontsize',10)

subplot(3,1,3);
plot(theta3diff);
ylabel('THETA3 Error','fontsize',10)
title('MSE 3','fontsize',10)

