// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.swervedrivespecialties.swervelib.CanPort;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;

import static frc.robot.constants.CANConstants.*;

/** Add your docs here. */
public final class DrivetrainConstants {
    public static final CanPort PIGEON_ID = new CanPort(14,CANIVORE_NAME);
    public static final GearRatio SWERVE_GEAR_RATIO = GearRatio.L2;
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;

    public static final class DrivetrainGeometry {

        /**
         * The left-to-right distance between the drivetrain wheels Should be measured
         * from center to center.
         */
        public static final double TRACKWIDTH_METERS = .585;
        /**
         * The front-to-back distance between the drivetrain wheels. Should be measured
         * from center to center.
         */
        public static final double WHEELBASE_METERS = .585;

        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight
         * line.
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = GlobalConstants.FALCON_500_MAX_RPM / 60.0
                * SdsModuleConfigurations.MK4I_L2.getDriveReduction()
                * SdsModuleConfigurations.MK4I_L2.getWheelDiameter()
                * Math.PI;
                //* .69;
        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
                / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);
    }

    public static final class FrontLeftSwerveConstants {
        public static final CanPort STEER_MOTOR_ID = new CanPort(FRONT_LEFT_STEER_MOTOR_ID,CANIVORE_NAME);
        public static final CanPort DRIVE_MOTOR_ID = new CanPort(FRONT_LEFT_DRIVE_MOTOR_ID,CANIVORE_NAME);
        public static final CanPort ENCODER_ID = new CanPort(FRONT_LEFT_ENCODER_ID, CANIVORE_NAME);
        public static final double ENCODER_OFFSET_RADIANS =  -Math.toRadians(0);            
        public static final int STATES_INDEX = 0;
    }

    public static final class FrontRightSwerveConstants {
        public static final CanPort STEER_MOTOR_ID = new CanPort(FRONT_RIGHT_STEER_MOTOR_ID,CANIVORE_NAME);
        public static final CanPort DRIVE_MOTOR_ID = new CanPort(FRONT_RIGHT_DRIVE_MOTOR_ID,GlobalConstants.CANIVORE_NAME);
        public static final CanPort ENCODER_ID = new CanPort(FRONT_RIGHT_ENCODER_ID,GlobalConstants.CANIVORE_NAME);
        public static final double ENCODER_OFFSET_RADIANS = -Math.toRadians(0);
        public static final int STATES_INDEX = 1;
    }

    public static final class BackLeftSwerveConstants {
        public static final CanPort STEER_MOTOR_ID = new CanPort(BACK_LEFT_STEER_MOTOR_ID,GlobalConstants.CANIVORE_NAME);
        public static final CanPort DRIVE_MOTOR_ID = new CanPort(BACK_LEFT_DRIVE_MOTOR_ID,GlobalConstants.CANIVORE_NAME);
        public static final CanPort ENCODER_ID = new CanPort(BACK_LEFT_ENCODER_ID,GlobalConstants.CANIVORE_NAME);
        public static final double ENCODER_OFFSET_RADIANS = -Math.toRadians(0);
        public static final int STATES_INDEX = 2;
    }

    public static final class BackRightSwerveConstants {
        public static final CanPort STEER_MOTOR_ID = new CanPort(BACK_RIGHT_STEER_MOTOR_ID,GlobalConstants.CANIVORE_NAME);
        public static final CanPort DRIVE_MOTOR_ID = new CanPort(BACK_RIGHT_DRIVE_MOTOR_ID,GlobalConstants.CANIVORE_NAME);
        public static final CanPort ENCODER_ID = new CanPort(BACK_RIGHT_ENCODER_ID,GlobalConstants.CANIVORE_NAME);
        public static final double ENCODER_OFFSET_RADIANS =  -Math.toRadians(0);
        public static final int STATES_INDEX = 3;
    }

    // Turning the bot gains used by PIDControllers
    public static final class ThetaGains {
        public static final double kP = 4;
        public static final double kI = 0;
        public static final double kD = .05;
        public static final double kTurnToleranceRad = 0.05;
        public static final double kTurnRateToleranceRadPerS = .25;
    }

    // Driving the bot gains used by PIDControllers
    public static final class TranslationGains {
       // public static final double kP = 2.2956;
       public static final double kP = 0.15;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kA = 0.16089;
        public static final double kV = 2.2528;
        public static final double kS = 0.60483;
    }
    //

}
