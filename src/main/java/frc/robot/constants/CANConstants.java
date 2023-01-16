// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class CANConstants {

    public static final String CANIVORE_NAME = "Canivore";

  
        public static final int FRONT_LEFT_STEER_MOTOR_ID = 2;
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 3;
        public static final int FRONT_LEFT_ENCODER_ID = 4;
        public static final double ENCODER_OFFSET_RADIANS =  -Math.toRadians(87.2);            
        public static final int STATES_INDEX = 0;
    
    public static final class FrontRightSwerveConstants {
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 5;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 6;
        public static final int FRONT_RIGHT_ENCODER_ID = 7;
        public static final double ENCODER_OFFSET_RADIANS = -Math.toRadians(1.45);
        public static final int STATES_INDEX = 1;
    }

    public static final class BackLeftSwerveConstants {
        public static final int BACK_LEFT_STEER_MOTOR_ID = 8;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 9;
        public static final int BACK_LEFT_ENCODER_ID = 10;
        public static final double ENCODER_OFFSET_RADIANS = -Math.toRadians(87.7);
        public static final int STATES_INDEX = 2;
    }

    public static final class BackRightSwerveConstants {
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 11;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 12;
        public static final int BACK_RIGHT_ENCODER_ID = 13;
        public static final double ENCODER_OFFSET_RADIANS =  -Math.toRadians(312.5);
        public static final int STATES_INDEX = 3;
    }

}
