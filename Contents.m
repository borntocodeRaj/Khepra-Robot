%---------- Contents------------%
kh4Start(comId); %exemple : kh4Start('COM10')
success = kh4SetPosition(motor_left,motor_right);
success = kh4SetSpeed(speed_motor_left,speed_motor_right);
success = kh4ConfigurePID(P,I,D);
success = kh4ConfigureSpeedProfile(acc_inc, acc_div, min_speed_acc, min_speed_dec,max_speed);
success = kh4ResetEncoders;
success = kh4SetRGBLeds(lr,lg,lb,rr,rg,rb,br,bg,bb);
success = kh4SetSpeedOpenloopMode(left_motor,right_motor);
success = kh4ResetEncoders;
success = kh4ConfigUSSensors(config);
Status = kh4GetBatteryStatus (argument);
[Acc]=kh4Accelerometer;
[Gyr]=kh4Gyroscope;
[left_motor, right_motor] = kh4ReadEncodersPosition;
[speed_motor_left, speed_motor_right] = kh4ReadSpeed;
[US_values] = kh4GetUSValues;
[Values_sensors]= kh4ReadProximitySensors;
[Values_sensors]= kh4ReadAmbientLigthSensors;


%---------------------------------------------------
kh4End;