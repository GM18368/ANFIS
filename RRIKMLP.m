


%Inverse kinematics of a 3 link planar manipulator using a MLP
clear;
clc;
%Set Ik test type, can be circle, leftLine, rightLine, curve,
%leftCurve,square
testType = "circle";

%%
%Manipulator details
l1 = 10; % length of first arm
l2 = 7; % length of second arm
l3 = 5; % length of third arm

%Workspace generation,
theta1 = 0:0.025:pi; 
theta2 = 0:0.025:pi/2; 
theta3 = -pi/2:0.025:pi/2; 

[THETA1,THETA2,THETA3] = meshgrid(theta1,theta2,theta3);

%%
%FK calculations
FKX = (l1 * cos(THETA1)) + (l2 * cos(THETA1 + THETA2)) + (l3 * cos(THETA1 + THETA2 + THETA3));
FKY = (l1 * sin(THETA1)) + (l2 * sin(THETA1 + THETA2)) + (l3 * sin(THETA1 + THETA2 + THETA3));

phi = THETA1 + THETA2 + THETA3; % Where does this get used?

%%
%create an array of all values and uniquetol it to reduce XY density in
%certain areas
fullArray = [FKX(:), FKY(:), phi(:), THETA1(:), THETA2(:), THETA3(:)];

c=fullArray(:,1:2);
[~,idx]=uniquetol(c,0.015,'ByRows',true);
sortedArray=fullArray(idx,:);


%Update variables with sorted array
FKX = sortedArray(:,1);
FKY = sortedArray(:,2);
phi = sortedArray(:,3);
THETA1 = sortedArray(:,4);
THETA2 = sortedArray(:,5);
THETA3 = sortedArray(:,6);
%%
%plot generated workspace
figure(1);
plot(FKX(:),FKY(:),'.');
title('Generated workspace')
grid on;
xlabel('x')
ylabel('y')
xlim([-25 25]);
ylim([-15 25]);
hold on;
 

%Join data sets and transpose
Input = [FKX(:), FKY(:), phi(:)]';

%Join the sets & transpose
Output = [THETA1(:), THETA2(:), THETA3(:)]';


%% Network setup & training
net = feedforwardnet([13 12 12],'trainlm'); %My attempt (Way more accurate than one layer)
%net = feedforwardnet([100],'trainlm'); %Paper suggested
net.divideParam.trainRatio = 0.7; % training set ratio
net.divideParam.valRatio = 0.15; % validation set ratio
net.divideParam.testRatio = 0.15; % test set ratio
net.trainParam.goal = 1e-6 ; %Set performance error goal
net.trainParam.min_grad = 1e-6/100; %Set minimum gradient
% train a neural network
net.trainParam.epochs = 4000; %Maximum number of epochs
%Train
net = train(net,Input,Output);


%% Validation tests, can rerun from here and just change testType
%straight line right side test
if testType == "rightLine"
    X = linspace(12,0,30); % x coordinates for validation
    Y(1:30) = 14; % y coordinates for validation
    PHI(1:30) = pi/3; % phi for validation
end

%straight line left side test
if testType == "leftLine"
    X = linspace(-14,3,30);  
    Y(1:30) = 14; 
    PHI(1:30) = pi; 
end

%Curve through workspace test
if testType == "curve"
    X = linspace(15,-15,30);
    Y = 17 - abs(X.^2)/30;
    PHI = linspace(0.5,2.5,30);
end

%curve on left side side
if testType == "leftCurve"
    Y = linspace(0,15,30);
    X = -18 + abs(Y.^2)/40;
    PHI(1:30) = pi;
end

% %circle in workspace test
if testType == "circle"
    angle = linspace(0,pi,30);
    X = -15 + 1.5*cos(2*angle);
    Y = -5 + 1.5*sin(2*angle);
    PHI(1:30) = 3.6;
end

% %circle in workspace test
if testType == "square"
   X = [linspace(0,5,10) linspace(5,0,10)];
   Y(1:10) = 16;
   Y(11:20) = 18;
   X(21:25) = 0;
   X(26:30) = 5;
   Y(21:25) = linspace(16,18,5);
   Y(26:30) = linspace(16,18,5);
   PHI(1:30) = pi/2;
end


%%
%Plot test ik positions
figure(2);
plot(X(:),Y(:),'-b');
title('Comparison of target and calculated IK')
xlabel('x')
ylabel('y')
hold on;

XYPHI = [X(:) , Y(:), PHI(:)]';
test = net(XYPHI);


%%
% %Actual IK calculations, to check if reachable demand.
% a = Y - (l3*sin(PHI)); % Y of wrist
% b = X - (l3*cos(PHI)); % X of wrist
% %Get theta2
% D = ( ((a.^2)+(b.^2) - l1^2 - l2^2)/(2*l1*l2)); %Costheta2
% THETA2D = real(acos(D));
% %Get theta1
% k1 = l2*sin(THETA2D);
% k2 = l1 + l2*cos(THETA2D);
% THETA1D = atan2((a.*k2)-(k1.*b),(a.*k1)+(b.*k2));
% %get theta 3
% THETA3D = PHI - (THETA1D + THETA2D);
%Plot the analytically calculated IK X&Y 
% valX = (l1 * cos(THETA1D(:))) + (l2 * cos(THETA1D(:)+ THETA2D(:))) + (l3 * cos(THETA1D(:)+THETA2D(:)+THETA3D(:)));
% valY = (l1 * sin(THETA1D(:))) + (l2 * sin(THETA1D(:)+ THETA2D(:))) + (l3 * sin(THETA1D(:)+THETA2D(:)+THETA3D(:)));
% plot(valX(:),valY(:),'-g');



%%
%test if the workspace looks alright
testX = (l1 * cos(test(1,:))) + (l2 * cos(test(1,:)+ test(2,:))) + (l3 * cos(test(1,:)+test(2,:)+test(3,:)));
testY = (l1 * sin(test(1,:))) + (l2 * sin(test(1,:)+ test(2,:))) + (l3 * sin(test(1,:)+test(2,:)+test(3,:)));
%phi = THETA1 + THETA2 + THETA3; % Where does this get used?
plot(testX(:),testY(:),'or','MarkerSize',2);
legend('Target','MLP');
hold off;


%%
%Error as a percentage of reach radius,Calc distance between target and calculated
error = sqrt((X(:)-testX(:)).^2 + (Y(:)-testY(:)).^2);
%convert to a percentage of reach radius (22)

percError = error/22 * 100;
figure(3)
plot(percError);
ylabel('Error percentage','fontsize',10)
xlabel('Comparison number','fontsize',10)
title('Error as a percentage of reach radius (22)','fontsize',10)


