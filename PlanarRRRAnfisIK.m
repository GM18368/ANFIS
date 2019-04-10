
%Inverse kinematics of a 3 link planar manipulator using Anfis
clear;
clc;

%Set Ik test type, can be leftCircle, rightCircle, leftLine, rightLine, curve,
%leftCurve,square
testType = "rightCircle";

%number of epochs for each system
numEpochs1 = 300;
numEpochs2 = 300;
numEpochs3 = 200;

%workspace resolution, this adjusts density
workSpaceRes = 0.02;%0.02 


%RRR planar manipulator IK with Anfis
l1 = 10; % length of first arm
l2 = 7; % length of second arm
l3 = 5; % length of third arm


%%
%workspace generation
theta1 = 0:0.025:pi; % all possible theta1 values
theta2 = 0:0.1:pi/2; % all possible theta2 values
theta3 = -pi/2:0.1:pi/2;% all possible theta2 values

%  theta1 = 0:0.1:pi; % all possible theta1 values
%  theta2 = 0:0.1:pi/2; % all possible theta2 values
%  theta3 = -pi/2:0.1:pi/2;% all possible theta2 values

[THETA1,THETA2,THETA3] = meshgrid(theta1,theta2,theta3);

%%
%FK calculations
FKX = (l1 * cos(THETA1)) + (l2 * cos(THETA1 + THETA2)) + (l3 * cos(THETA1 + THETA2 + THETA3));
FKY = (l1 * sin(THETA1)) + (l2 * sin(THETA1 + THETA2)) + (l3 * sin(THETA1 + THETA2 + THETA3));
PHI = THETA1 + THETA2 + THETA3; %one paper uses this, one does not?


%%
%create an array of all values and uniquetol it for xy and for phi

fullArray = [FKX(:), FKY(:), PHI(:), THETA1(:), THETA2(:), THETA3(:)];

c=fullArray(:,1:2); %sorts by x,y locations that aren't too close together

[~,idx]=uniquetol(c,workSpaceRes,'ByRows',true); 
sortedArray=fullArray(idx,:);


%Set net variables since we don't want to change all the code

FKX = sortedArray(:,1);
FKY = sortedArray(:,2);
PHI = sortedArray(:,3);
THETA1 = sortedArray(:,4);
THETA2 = sortedArray(:,5);
THETA3 = sortedArray(:,6);



%%
%Plot the generated workspace?
figure(1);
plot(FKX(:), FKY(:),'.');
title('Generated workspace')
grid on;
xlabel('x')
ylabel('y')
xlim([-25 25]);
ylim([-15 25]);
hold on;



%%
%genfis datasets
data1 = [FKX(:) FKY(:)  PHI(:)]; % create x-y-phi-theta1 dataset
data2 = [FKX(:) FKY(:)  PHI(:)]; % create x-y-phi-theta2 dataset
data3 = [FKX(:) FKY(:)  PHI(:)];% create x-y-phi-theta3 dataset
%anfis datasets
fulldata1 = [FKX(:) FKY(:)  PHI(:) THETA1(:)];
fulldata2 = [FKX(:) FKY(:)  PHI(:) THETA2(:)];
fulldata3 = [FKX(:) FKY(:)  PHI(:) THETA3(:)];

%%
%Initial genfis gridpartition
genOpt = genfisOptions('GridPartition');
%gridpartition options
genOpt.NumMembershipFunctions = [4 4 2]; %membership function number for inputs x y and phi as suggested by paper 2
genOpt.InputMembershipFunctionType = ["gaussmf" "gaussmf" "gaussmf" ];
genOpt.OutputMembershipFunctionType = "linear";

%%
%set number of epochs for anfis training

%First genfis
disp('--> first GENFIS.')
inFIS1 = genfis(data1,THETA1(:),genOpt);

% %Second genfis
disp('--> second GENFIS.')
inFIS2 = genfis(data2,THETA2(:),genOpt);

% %third genfis 
disp('--> third GENFIS.')
inFIS3 = genfis(data3,THETA3(:),genOpt);

%%
%anfis setup & suppress outputs
opt1 = anfisOptions('InitialFIS',inFIS1);
% opt1 = anfisOptions;
opt1.DisplayANFISInformation = 0;
opt1.DisplayErrorValues = 0;
opt1.DisplayStepSize = 0;
opt1.DisplayFinalResults = 0;
opt1.ErrorGoal = 1e-4;
%opt1.InitialFIS = 3;
%opt1.ValidationData = checkdata1;
%anfis2
opt2 = anfisOptions('InitialFIS',inFIS2);
%opt2 = anfisOptions;
opt2.DisplayANFISInformation = 0;
opt2.DisplayErrorValues = 0;
opt2.DisplayStepSize = 0;
opt2.DisplayFinalResults = 0;
opt2.ErrorGoal = 1e-4; 
%opt2.InitialFIS = 4;
%opt2.ValidationData = checkdata2;
%anfis3
opt3 = anfisOptions('InitialFIS',inFIS3);
%opt3 = anfisOptions;
opt3.DisplayANFISInformation = 0;
opt3.DisplayErrorValues = 0;
opt3.DisplayStepSize = 0;
opt3.DisplayFinalResults = 0;
opt3.ErrorGoal = 1e-4; 
%opt3.InitialFIS = 3; %only for use with just anfis
%opt3.ValidationData = checkdata3;


%%
% Train anfis1.
disp('--> Training first ANFIS network.')
opt1.EpochNumber = numEpochs1;
anfis1 = anfis(fulldata1,opt1);


%Train anfis2
disp('--> Training second ANFIS network.')

opt2.EpochNumber = numEpochs2;
anfis2 = anfis(fulldata2,opt2);

%train anfis3
disp('--> Training third ANFIS network.')
opt3.EpochNumber = numEpochs3;
anfis3 = anfis(fulldata3,opt3);

%% Validation tests 
%straight line right side test
if testType == "rightLine"
    X = linspace(12,0,30); % x coordinates for validation
    Y(1:30) = 14; % y coordinates for validation
    phiV(1:30) = pi/3; % phi for validation
end

%straight line left side test
if testType == "leftLine"
    X = linspace(-14,3,30);  
    Y(1:30) = 14; 
    phiV(1:30) = pi; 
end

%Curve through workspace test
if testType == "curve"
    X = linspace(15,-15,30);
    Y = 17 - abs(X.^2)/30;
    phiV = linspace(0.5,3,30);
end

%curve on left side side
if testType == "leftCurve"
    Y = linspace(0,15,30);
    X = -18 + abs(Y.^2)/40;
    phiV(1:30) = 0;
end

% %circle in workspace test
if testType == "leftCircle"
    angle = linspace(0,pi,30);
    X = -15 + 1.5*cos(2*angle);
    Y = -5 + 1.5*sin(2*angle);
    phiV(1:30) = 3.6;
    
end

% %circle in workspace test
if testType == "rightCircle"
    angle = linspace(0,pi,30);
    X = 10 + 1.5*cos(2*angle);
    Y = 16 + 1.5*sin(2*angle);
    phiV(1:30) = pi/3;
    
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
   phiV(1:30) = pi/1.5;
end

%%
%plot validation xy coordinates
figure(2);
plot(X,Y,'-b');
title('Comparison of target and anfis IK)','fontsize',10);
%xlim([-25 25]);
%ylim([-15 25]);
hold on;

% %%
% %Actual IK calculations, for comparison with anfis
% a = Y - (l3*sin(phiV)); % Y of wrist
% b = X - (l3*cos(phiV)); % X of wrist
% %Get theta2
% D = ( ((a.^2)+(b.^2) - l1^2 - l2^2)/(2*l1*l2)); %Costheta2
% THETA2D = real(acos(D));
% %Get theta1
% k1 = l2*sin(THETA2D);
% k2 = l1 + l2*cos(THETA2D);
% THETA1D = atan2((a.*k2)-(k1.*b),(a.*k1)+(b.*k2));
% %get theta 3
% THETA3D = phiV - (THETA1D + THETA2D);
% 
% %Plot the calculated IK X&Y positions, this is green, can be used to test
% %if its a reasonable demand of the system
% valX = (l1 * cos(THETA1D)) + (l2 * cos(THETA1D+ THETA2D)) + (l3 * cos(THETA1D+THETA2D+THETA3D));
% valY = (l1 * sin(THETA1D)) + (l2 * sin(THETA1D+ THETA2D)) + (l3 * sin(THETA1D+THETA2D+THETA3D));
% plot(valX(:),valY(:),'-g');

%%
%evaluate anfis for test coordinates

XY = [X' Y' phiV'];

THETA1P = evalfis(XY,anfis1); % theta1 predicted by anfis1
THETA2P = evalfis(XY,anfis2); % theta2 predicted by anfis2
THETA3P = evalfis(XY,anfis3); % theta3 predicted by anfis3


testX = (l1 * cos(THETA1P)) + (l2 * cos(THETA1P+ THETA2P)) + (l3 * cos(THETA1P+THETA2P+THETA3P));
testY = (l1 * sin(THETA1P)) + (l2 * sin(THETA1P+ THETA2P)) + (l3 * sin(THETA1P+THETA2P+THETA3P));
plot(testX,testY,'or','MarkerSize',2);
legend('Target','ANFIS');
hold off;

%%
%Error as a percentage of reach radius,Calc distance between target and calculated
error = sqrt((X(:)-testX(:)).^2 + (Y(:)-testY(:)).^2);
%convert to a percentage of reach radius (22)
percError = error/22 * 100;
%plot the error
figure(3)
plot(percError);
ylabel('Error percentage','fontsize',10);
xlabel('EE position number');
title('Error as a percentage of reach radius (22)','fontsize',10);


