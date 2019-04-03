%Neural network RR IK, gonna try with two first
%Inputs are the X&Y
%Desired outputs are the actual calculated thetas


%Clear the environment
clear;
clc;
%RRR planar manipulator IK
l1 = 10; % length of first arm
l2 = 7; % length of second arm
l3 = 5; % length of third arm

%these arrays need to be the same size, resolution here affects anfis
%accuracy, these are all increasing with each other so may not be relective
%of workspace?
%theta1 = 0:0.025:pi; % all possible theta1 values
%theta2 = 0:0.0125:pi/2; % all possible theta2 values
%theta3 = -pi/2:0.025:pi/2; % all possible theta3 values

%Generate workspace data, want a mix of values
i=1;
for theta1 = 0:pi/10:pi
    
    for theta2 = 0:pi/20:pi/2
         
        for theta3 = -pi/2:pi/10:pi/2
            THETA1(i) = theta1;
            THETA2(i) = theta2;
            THETA3(i) = theta3;
            i = i+1;
           
            
        end
    end
end

%%
%FK calculations
FKX = (l1 * cos(THETA1)) + (l2 * cos(THETA1 + THETA2)) + (l3 * cos(THETA1 + THETA2 + THETA3));
FKY = (l1 * sin(THETA1)) + (l2 * sin(THETA1 + THETA2)) + (l3 * sin(THETA1 + THETA2 + THETA3));

phi = THETA1 + THETA2 + THETA3; % Where does this get used?

%  figure(1);
%  plot(FKX,FKY,'.');
%  title('Generated workspace')
%  xlabel('x')
%  ylabel('y')


%Join data sets and transpose
Input = [FKX(:), FKY(:), phi(:)]';

%Join the sets & transpose
Output = [THETA1(:), THETA2(:), THETA3(:)]';


%% Network setup & training
net = feedforwardnet([8 8]); %WAS 13 13
net.divideParam.trainRatio = 0.6; % training set was 0.7
net.divideParam.valRatio = 0.2; % validation set 
net.divideParam.testRatio = 0.2; % test set 
net.trainParam.goal = 1e-15 ; %Set error goal
net.trainParam.min_grad = 1e-15/100;
% train a neural network
net.trainParam.epochs = 3000;
%Train
net = train(net,Input,Output);



%%
%Some test coordinates, Don't know if we need these?
X = 10:0.1:15; % x coordinates for validation
Y = 15:-0.1:10; % y coordinates for validation, This is just a line currently
PHI(1,51) = 0; % y coordinates for validation, This is just a line currently
%Plot test ik
figure(2);
plot(X(:),Y(:),'-b');
xlim([-25 25]);
ylim([0 25]);
title('Test IK')
xlabel('x')
ylabel('y')
hold on;


%Actual IK calculations, for comparison with anfis
%These are incorrect at the minute


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


XY = [X(:) , Y(:), PHI(:)]';
%Then we need to compare this to calculated values
test = net(XY);

%%Plot the analytically calculated IK X&Y 
valX = (l1 * cos(THETA1D(:))) + (l2 * cos(THETA1D(:)+ THETA2D(:))) + (l3 * cos(THETA1D(:)+THETA2D(:)+THETA3D(:)));
valY = (l1 * sin(THETA1D(:))) + (l2 * sin(THETA1D(:)+ THETA2D(:))) + (l3 * sin(THETA1D(:)+THETA2D(:)+THETA3D(:)));
plot(valX(:),valY(:),'-g');



%%
%test if the workspace looks alright
testX = (l1 * cos(test(1,:))) + (l2 * cos(test(1,:)+ test(2,:))) + (l3 * cos(test(1,:)+test(2,:)+test(3,:)));
testY = (l1 * sin(test(1,:))) + (l2 * sin(test(1,:)+ test(2,:))) + (l3 * sin(test(1,:)+test(2,:)+test(3,:)));
%phi = THETA1 + THETA2 + THETA3; % Where does this get used?
 plot(testX(:),testY(:),'-r');
hold off;

%%
%Extract the networks calculated thetas and compare, these aren't the right
%metrics
theta1error = THETA1D - test(1,:);
theta2error = THETA2D - test(2,:);
theta3error = THETA3D - test(3,:);



figure(3);
subplot(3,1,1);
plot(theta1error);
ylabel('Theta1 Error','fontsize',10)
title('MSE 1','fontsize',10)

subplot(3,1,2);
plot(theta2error);
ylabel('Theta2 Error','fontsize',10)
title('MSE 2','fontsize',10)

subplot(3,1,3);
plot(theta3error);
ylabel('Theta3 Error','fontsize',10)
title('MSE 3','fontsize',10)

