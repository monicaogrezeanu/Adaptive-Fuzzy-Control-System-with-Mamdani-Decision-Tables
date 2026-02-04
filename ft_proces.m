motor_R = 1.4;
motor_L = 0.002;
motor_J = 0.2;
motor_D = 0.0001;
motor_km = 0.3;
motor_ke = 0.25;
motor_num = [motor_km]
motor_den = [(motor_L*motor_J) (motor_L*motor_D+motor_R*motor_J) (motor_R*motor_D+motor_km*motor_ke)]
