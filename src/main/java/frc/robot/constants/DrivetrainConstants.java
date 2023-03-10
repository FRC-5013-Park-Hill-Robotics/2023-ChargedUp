// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.falconSwerveLib.util.COTSFalconSwerveConstants;
import frc.falconSwerveLib.util.CanPort;
import frc.falconSwerveLib.util.SwerveModuleConstants;

import static frc.robot.constants.CANConstants.*;

/** Add your docs here. */
public final class DrivetrainConstants {
    public DrivetrainConstants(double kp, double ki, double kd) {
    }

    public static final CanPort PIGEON_ID = new CanPort(CANConstants.PIGEON_ID,CANIVORE_NAME);
    //public static final GearRatio SWERVE_GEAR_RATIO = GearRatio.L2;
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;

    public static final int pigeonID = 1;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule =  
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = 0.42; 
    public static final double wheelBase = 0.42; 
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics 
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
     public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    public static final double kA = 0.16089;
    public static final double kV = 2.2528;
    public static final double kS = 0.60483;
    /* Drive Motor Characterization Values 
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (kS / 12); 
    public static final double driveKV = (kV / 12);
    public static final double driveKA = (kA / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed =4.968; //TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 12.01; //TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { 
        public static final CanPort driveMotorID = new CanPort(FRONT_LEFT_DRIVE_MOTOR_ID, CANIVORE_NAME);
        public static final CanPort angleMotorID = new CanPort(FRONT_LEFT_STEER_MOTOR_ID, CANIVORE_NAME);
        public static final CanPort canCoderID = new CanPort(FRONT_LEFT_ENCODER_ID, CANIVORE_NAME);
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(66);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { 
        public static final CanPort driveMotorID = new CanPort(FRONT_RIGHT_DRIVE_MOTOR_ID, CANIVORE_NAME);
        public static final CanPort angleMotorID = new CanPort(FRONT_RIGHT_STEER_MOTOR_ID, CANIVORE_NAME);
        public static final CanPort canCoderID = new CanPort(FRONT_RIGHT_ENCODER_ID, CANIVORE_NAME);
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(59);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    /* Back Left Module - Module 2 */
    public static final class Mod2 { 
        public static final CanPort driveMotorID = new CanPort(BACK_LEFT_DRIVE_MOTOR_ID, CANIVORE_NAME);
        public static final CanPort angleMotorID = new CanPort(BACK_LEFT_STEER_MOTOR_ID, CANIVORE_NAME);
        public static final CanPort canCoderID = new CanPort(BACK_LEFT_ENCODER_ID, CANIVORE_NAME);
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(147.9);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { 
        public static final CanPort driveMotorID = new CanPort(BACK_RIGHT_DRIVE_MOTOR_ID, CANIVORE_NAME);
        public static final CanPort angleMotorID = new CanPort(BACK_RIGHT_STEER_MOTOR_ID, CANIVORE_NAME);
        public static final CanPort canCoderID = new CanPort(BACK_RIGHT_ENCODER_ID, CANIVORE_NAME);
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(337);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }


/* 
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
*/
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
