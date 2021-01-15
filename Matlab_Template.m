Simulink.importExternalCTypes(which('kortex_wrapper_data.h'));
gen3Kinova = kortex();
%*******Change the IP_ADDRESS to connect to the robot*******
gen3Kinova.ip_address = 'xxx.xxx.xxx.xxx';
%*******Add the username******* 
gen3Kinova.user = 'xxxxxxx';
%*******Add the password*******
gen3Kinova.password = 'xxxxxxx';

%connect to the robot
isOk = gen3Kinova.CreateRobotApisWrapper();
if isOk
   disp('You are connected to the robot!'); 
else
   error('Failed to establish a valid connection!'); 
end

%*******Write the Read_CSV function*******
%*******Start here*******








%*******Function end here*******

[angle, position, orientation, gripper_position, translation_speed, action_sequence]= Read_CSV('Address of the CSV file');


%*******create cartesian command, by moving position and orientation matrix side by side*******


%*******Write the loop that perform the action*******
%*******start here*******


	
	
	
	
	
	
%*******end the loop here*******



%*******Write code that simulate the motion of the robot in real time*******
%*******start here*******






%*******end here*******

%discounting the robot
isOk = gen3Kinova.DestroyRobotApisWrapper();
