%Clear the environment
clear;
clc;
%RRR planar manipulator IK
l1 = 10; % length of first arm
l2 = 7; % length of second arm
l3 = 5; % length of third arm

% %Data generation
% i=1;
% for theta1 = 0:pi/40:pi
%     
%     for theta2 = 0:pi/40:pi/2
%          
%         for theta3 = -pi/2:pi/40:pi/2 %used to be from -pi/2
%             THETA1(i) = theta1;
%             THETA2(i) = theta2;
%             THETA3(i) = theta3;
%             i = i+1;
%            
%             
%         end
%     end
% end
%%
%alternate generation
theta1 = 0:0.4:pi; % all possible theta1 values
theta2 = 0:0.2:pi/2; % all possible theta2 values
theta3 = -pi/2:0.4:pi/2;
[THETA1,THETA2,THETA3] = meshgrid(theta1,theta2,theta3);

%%
%FK calculations
FKX = (l1 * cos(THETA1)) + (l2 * cos(THETA1 + THETA2)) + (l3 * cos(THETA1 + THETA2 + THETA3));
FKY = (l1 * sin(THETA1)) + (l2 * sin(THETA1 + THETA2)) + (l3 * sin(THETA1 + THETA2 + THETA3));
PHI = THETA1 + THETA2 + THETA3; % Where does this get used?

%Plot the generated workspace?
figure(1);
plot(FKX(:), FKY(:),'.');
hold on;




%genfis datasets
data1 = [FKX(:) FKY(:)  PHI(:) ]; % create x-y-phi-theta1 dataset
data2 = [FKX(:) FKY(:)  PHI(:) ]; % create x-y-phi-theta2 dataset
data3 = [FKX(:) FKY(:)  PHI(:) ];% create x-y-phi-theta3 dataset
%anfis datasets
fulldata1 = [FKX(:) FKY(:)  PHI(:) THETA1(:) ];
fulldata2 = [FKX(:) FKY(:)  PHI(:) THETA2(:) ];
fulldata3 = [FKX(:) FKY(:)  PHI(:) THETA3(:) ];

%%
%Initial genfis gridpartition
genOpt = genfisOptions('GridPartition');

%initial genfis subtractive
%genOpt = genfisOptions('SubtractiveClustering');
%genOpt.NumClusters = 6;

%Initial genfis clustering?
 %genOpt = genfisOptions('FCMClustering','FISType','sugeno');
 %genOpt2 = genfisOptions('FCMClustering','FISType','sugeno');
 %genOpt3 = genfisOptions('FCMClustering','FISType','sugeno');

%gridpartition options
 genOpt.NumMembershipFunctions = [3 3 9]; %membership function number for inputs x y and phi
 genOpt.InputMembershipFunctionType = ["gbellmf" "gbellmf" "gbellmf"];
 genOpt.OutputMembershipFunctionType =["linear"];

%clustering options
%genOpt.NumClusters = 8; %Essentially the number of membership functions.
%genOpt.Verbose = 0; %Suppress output
% genOpt2.NumClusters = 12; %Essentially the number of membership functions.
% genOpt2.Verbose = 0; %Suppress output
% genOpt3.NumClusters = 12; %Essentially the number of membership functions.
% genOpt3.Verbose = 0; %Suppress output

%set number of epochs for anfis training
numEpochs = 10;

%genOpt.InputMembershipFunctionType = 'gaussmf';
disp('--> generating first GENFIS.')
inFIS1 = genfis(data1,THETA1(:),genOpt);
disp('--> generating second GENFIS.')
inFIS2 = genfis(data2,THETA2(:),genOpt);
disp('--> generating third GENFIS.')
inFIS3 = genfis(data3,THETA3(:),genOpt);

%%
%anfis setup
%opt = anfisOptions;
%opt.InitialFIS = 6;
opt1 = anfisOptions('InitialFIS',inFIS1);
opt1.DisplayANFISInformation = 0;
opt1.DisplayErrorValues = 0;
opt1.DisplayStepSize = 0;
opt1.DisplayFinalResults = 0;

%anfis2
opt2 = anfisOptions('InitialFIS',inFIS2);
opt2.DisplayANFISInformation = 0;
opt2.DisplayErrorValues = 0;
opt2.DisplayStepSize = 0;
opt2.DisplayFinalResults = 0;
%anfis3
opt3 = anfisOptions('InitialFIS',inFIS3);
opt3.DisplayANFISInformation = 0;
opt3.DisplayErrorValues = 0;
opt3.DisplayStepSize = 0;
opt3.DisplayFinalResults = 0;




%%
% Train anfis1.
disp('--> Training first ANFIS network.')
opt1.EpochNumber = numEpochs;
anfis1 = anfis(fulldata1,opt1);


%Train anfis2
disp('--> Training second ANFIS network.')
% opt.InitialFIS = 5;
opt2.EpochNumber = numEpochs;
anfis2 = anfis(fulldata2,opt2);

%train anfis3
disp('--> Training third ANFIS network.')
% opt.InitialFIS = 4;
opt3.EpochNumber = numEpochs;
anfis3 = anfis(fulldata3,opt3);

%%
%straight line right side
 % x coordinates for validation
% X = 12:-0.1:0; % x coordinates for validation
% Y(1:121) = 14; % y coordinates for validation
% phiV(1:121) = 0; % phi for validation

%%
%straight line left side
X = -13:0.1:5; % x coordinates for validation
Y(1:181) = 14 % y coordinates for validation
phiV(1:181) = pi/2; % phi for validation




% figure(2);
plot(X,Y,'-b');
title('Comparison of target and anfis IK)','fontsize',10);
xlim([-25 25]);
ylim([0 25]);
% hold on;
%Actual IK calculations, for comparison with anfis


a = Y - (l3*sin(phiV)); % Y of wrist
b = X - (l3*cos(phiV)); % X of wrist

%Get theta2

D = ( ((a.^2)+(b.^2) - l1^2 - l2^2)/(2*l1*l2)); %Costheta2


THETA2D = real(acos(D));

%Get theta1

k1 = l2*sin(THETA2D);
k2 = l1 + l2*cos(THETA2D);


THETA1D = atan2((a.*k2)-(k1.*b),(a.*k1)+(b.*k2));

%get theta 3
THETA3D = phiV - (THETA1D + THETA2D);
%%
%Plot the calculated IK X&Y positions, this is green
valX = (l1 * cos(THETA1D)) + (l2 * cos(THETA1D+ THETA2D)) + (l3 * cos(THETA1D+THETA2D+THETA3D));
valY = (l1 * sin(THETA1D)) + (l2 * sin(THETA1D+ THETA2D)) + (l3 * sin(THETA1D+THETA2D+THETA3D));
plot(valX(:),valY(:),'-g');


XY = [X' Y' phiV'];
THETA1P = evalfis(XY,anfis1); % theta1 predicted by anfis1
THETA2P = evalfis(XY,anfis2); % theta2 predicted by anfis2
THETA3P = evalfis(XY,anfis3); % theta3 predicted by anfis3


testX = (l1 * cos(THETA1P)) + (l2 * cos(THETA1P+ THETA2P)) + (l3 * cos(THETA1P+THETA2P+THETA3P));
testY = (l1 * sin(THETA1P)) + (l2 * sin(THETA1P+ THETA2P)) + (l3 * sin(THETA1P+THETA2P+THETA3P));
plot(testX,testY,'-r');
hold off;

%%
%Error as a percentage of reach radius,Calc distance between target and calculated
error = sqrt((X(:)-testX(:)).^2 + (Y(:)-testY(:)).^2);
%convert to a percentage of reach radius (22)

percError = error/22 * 100;
figure(3)
plot(percError);
ylabel('Error percentage','fontsize',10);
xlabel('EE position number');
title('Error as a percentage of reach radius (22)','fontsize',10);





%%
%mean squared error
% theta1diff = (THETA1D(:) - THETA1P).^2; 
% theta2diff = (THETA2D(:) - THETA2P).^2;
% theta3diff = (THETA3D(:) - THETA3P).^2;

% %Original metrics ,Is this a good measure of error?
% theta1diff = THETA1D(:) - THETA1P; 
% theta2diff = THETA2D(:) - THETA2P;
% theta3diff = THETA3D(:) - THETA3P;

%Plot the errors for each theta
% figure(3);
% subplot(3,1,1);
% plot(theta1diff);
% ylabel('Theta1 Error','fontsize',10)
% title('MSE 1','fontsize',10)
% 
% subplot(3,1,2);
% plot(theta2diff);
% ylabel('Theta2 Error','fontsize',10)
% title('MSE 2','fontsize',10)
% 
% subplot(3,1,3);
% plot(theta3diff);
% ylabel('Theta3 Error','fontsize',10)
% title('MSE 3','fontsize',10)

