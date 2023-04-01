// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class GlobalConstants {
    public static final int PCM_ID = 1;
    public static final int FALCON_500_MAX_RPM = 6380;
    public static final int STATUS_LED_PWM_PORT = 1;
	public static final int PNEUMATICS_HUB = 0;
	public static final String CANIVORE_NAME = "Canivore";


    public static final class ControllerConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
		public static final int OPERATOR_CONTROLLER_PORT = 1;
		public static final int PROGRAMMER_CONTROLLER_PORT = 2;
        public static final double DEADBAND = 0.05;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR = 20;
        public static final int DROP_INTAKE_SOLENOID_CHANNEL = 1;
        public static final int RAISE_INTAKE_SOLENOID_CHANNEL = 0;
		public static final double INTAKE_SPEED = 0.5;
        public static final double CUBE_INTAKE_SPEED = 0.6;
        public static final int ROLLER_SERVO = 0;

        public static final Rotation2d WRIST_OFFSET_DEGREES = Rotation2d.fromDegrees(-27);
        public static final Rotation2d TARGET_WRIST_ANGLE = Rotation2d.fromDegrees(22);
        public static final Rotation2d WRIST_TOLERANCE = Rotation2d.fromDegrees(2);

        public static final double kP = 1.3;
        public static final double kI = 0;
        public static final double kD = 0.05;
        public static final double kS = 0.8;
        public static final double kV = 1;
        public static final double kG = 1.1;
    }

    public static final class LEDConstants {
        public static int STATUS_LED_PWM_PORT = 0;
    }


}