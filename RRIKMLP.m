clc;
clear;
%RRR planar manipulator IK with a MLP.
l1 = 10; % length of first arm
l2 = 7; % length of second arm
l3 = 5; % length of third arm


%Generate workspace data, want a mix of values
% i=1;
% for theta1 = 0:pi/10:pi
%     
%     for theta2 = 0:pi/60:pi/2
%          
%         for theta3 = -pi/2:pi/40:pi/2                   
%             THETA1(i) = theta1;
%             THETA2(i) = theta2;
%             THETA3(i) = theta3;
%             i = i+1;
%            
%             
%         end
%     end
% end
% 

%%
%alternate generation, Can we make this better? reduce duplicates?
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
%create an array of all values and unique it to reduce XY duplicates like the
%paper does?

fullArray = [FKX(:), FKY(:), phi(:), THETA1(:), THETA2(:), THETA3(:)];

c=fullArray(:,1:2); %sorts by unique x,y locations
[~,idx]=uniquetol(c,0.02,'ByRows',true);
sortedArray=fullArray(idx,:);




%Set net variables since we don't want to change all the code

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
ylim([-10 25]);
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
net.trainParam.epochs = 4000;
%Train
net = train(net,Input,Output);



%%
%Some test coordinates, Make sure these are possible! (You'll get an error
%with validation IK)

%%
%straight line right side test
% X = 12:-0.5:0; % x coordinates for validation
% Y(1:25) = 14 % y coordinates for validation
% PHI(1:25) = 0; % phi for validation

%%
%straight line left side test
% X = -14:0.5:3; % x coordinates for validation
% Y(1:35) = 14 % y coordinates for validation
% PHI(1:35) = pi; % phi for validation

%%
%Curve through workspace test
% X = 15:-1:-15;
% Y = 17 - abs(X.^2)/30;
% PHI = linspace(0.5,2.5,31);

%%
%curve on left side side
% Y = 0:0.5:15;
% X = -18 + abs(Y.^2)/40;
% PHI(1:31) = pi;
%%
%circle in workspace
angle = linspace(0,pi,50);
X = 5 + 1.5*cos(2*angle);
Y = 10 + 1.5*sin(2*angle);
PHI(1:50) = pi;

%%
%Plot test ik
% figure(2);
plot(X(:),Y(:),'-b');
% xlim([-25 25]);
% ylim([0 25]);
% title('Comparison of target and calculated IK')
% xlabel('x')
% ylabel('y')
hold on;

%%
% %Actual IK calculations, for comparison with anfis
% %These are incorrect at the minute
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


%%
XYPHI = [X(:) , Y(:), PHI(:)]';
%Then we need to compare this to calculated values
test = net(XYPHI);
%Plot the analytically calculated IK X&Y 
valX = (l1 * cos(THETA1D(:))) + (l2 * cos(THETA1D(:)+ THETA2D(:))) + (l3 * cos(THETA1D(:)+THETA2D(:)+THETA3D(:)));
valY = (l1 * sin(THETA1D(:))) + (l2 * sin(THETA1D(:)+ THETA2D(:))) + (l3 * sin(THETA1D(:)+THETA2D(:)+THETA3D(:)));
plot(valX(:),valY(:),'-g');



%%
%test if the workspace looks alright
testX = (l1 * cos(test(1,:))) + (l2 * cos(test(1,:)+ test(2,:))) + (l3 * cos(test(1,:)+test(2,:)+test(3,:)));
testY = (l1 * sin(test(1,:))) + (l2 * sin(test(1,:)+ test(2,:))) + (l3 * sin(test(1,:)+test(2,:)+test(3,:)));
%phi = THETA1 + THETA2 + THETA3; % Where does this get used?
plot(testX(:),testY(:),'or','MarkerSize',2);
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

%%
%Extract the networks calculated thetas and compare, these aren't the right
%metrics
% theta1error = THETA1D - test(1,:);
% theta2error = THETA2D - test(2,:);
% theta3error = THETA3D - test(3,:);



% figure(3);
% subplot(3,1,1);
% plot(theta1error);
% ylabel('Theta1 Error','fontsize',10)
% title('MSE 1','fontsize',10)
% 
% subplot(3,1,2);
% plot(theta2error);
% ylabel('Theta2 Error','fontsize',10)
% title('MSE 2','fontsize',10)
% 
% subplot(3,1,3);
% plot(theta3error);
% ylabel('Theta3 Error','fontsize',10)
% title('MSE 3','fontsize',10)

