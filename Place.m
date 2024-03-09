function[]=Place(motor3Obj,motor2Obj,motor1Obj,x,y,z)
% Defining Robotic Arm lengths
L1 = 50; L2 = 95; L3 = 185; L4 = 110;   

%Base Joint calculation
% Using inverse kinematic equations we calculate the target base 
% angle using the co-ordinates of the point we have to reach
    if (x<0 && y>=0)            %third quadrant
        theta3 = atan(y/x);
    elseif (x>=0 &&  y>=0)      %second quadrant
        theta3 = atan(y/x)-pi;
    end
 
% Elbow joint calculation
% Using inverse kinematic equations we calculate the target arm 
% angle using the co-ordinates of the point we have to reach
L1_ = sqrt((L1*L1) + (L2*L2) - 2*L1*L2*cos(3*pi/4));
alfa = (pi/2)- asin((L2*sin(3*pi/4))/L1_);
r = sqrt(L3*L3-(z+L4-L1_*sin(alfa))^2)-L1_*cos(alfa);
C =  atan(z/r);
L3_= sqrt((L1_*L1_) + (z*z) + (r*r)- 2*L1_*(sqrt(z*z + r*r))*cos(pi -( C + alfa)));
beta = acos(((L3*L3) +(L4*L4) - (L3_*L3_))/(2*L3*L4));
theta2 = (pi/2)-beta + alfa;
M = theta2 - (asin((L1*sin(3*pi/4))/L1_));
MInDegrees1 = rad2deg(M);
Mi=deg2rad(81); % Initial elbow joint angle at homing position


% Base motor control 
% rotation direction of base motor
if abs(rad2deg(theta3))>=abs((readRotation(motor3Obj))/3)
    sign = -1;  %anticlockwise
else
    sign = 1;   %clockwise
end

% The loop will keep running until the desired angle is reached
while 1
    motor3Obj.Speed=80*sign;
    % when the desired angle is reached, the following conditions becomes
    % true and the loop is broken
    if  sign*double(abs(readRotation(motor3Obj))/3)<=sign*abs(rad2deg(theta3))
       break;
    end
end   
% After reaching the desired position, we set the base motor speed to zero 
motor3Obj.Speed = 0; 
% We give a pause of half a second after reaching the desired position
pause(0.5)

% Elbow motor control loop

% The loop will keep running until the desired angle is reached
while 1
    motor2Obj.Speed=10;
    % when the desired angle is reached, the following conditions becomes
    % true and the loop is broken
    if  double(readRotation(motor2Obj)/5)>=rad2deg(Mi)-rad2deg(M) 
       break;
    end
end
% After reaching the desired position, we set the arm motor speed to zero   
motor2Obj.Speed = 0;
% We give a pause of half a second after reaching the desired position
pause(0.5)

% Opening the gripper
motor1Obj.Speed=-30; % Giving gripper motor a speed
start(motor1Obj);    % Starting the motor
pause(0.20)          % Waiting for the gripper to reach the open position
motor1Obj.Speed=0;     % stopping the gripper motor after reaching des. pos.

end