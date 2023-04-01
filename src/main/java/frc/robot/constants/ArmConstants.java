// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmConstants {
    public static final Rotation2d ARM_OFFSET_DEGREES = Rotation2d.fromDegrees(-270);
    public static final double SPOOL_DIAMETER = Units.inchesToMeters(2.0);
    public static final double DISTANCE_PER_SPOOL_REVOLUTION_METERS = SPOOL_DIAMETER * Math.PI;
    public static final double SPOOL_ROTATIONS_PER_METER = 1/DISTANCE_PER_SPOOL_REVOLUTION_METERS;
    public static final double FALCON_ROTATIONS_PER_SPOOL = 27.0;
    public static final double PULSES_PER_FALCON_ROTATION = 2048;
    public static final double PULSES_PER_METER_EXTENSION = SPOOL_ROTATIONS_PER_METER * FALCON_ROTATIONS_PER_SPOOL * PULSES_PER_FALCON_ROTATION;
    public static final double FULL_EXTENSION_DISTANCE = 0.966;

    public static final class ExtensionGains {
        public static final double kP = 35;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double kS = 0.25106;
        public static final double kV = 16.575;
        public static final double kA = 0.80452;
    }

    public static final class RotationGains {
        public static final double kGPercent = 0.075;
        public static final Rotation2d TOLERANCE= Rotation2d.fromDegrees(2.5);
        //public static final double kP = 8.0735;
        public static final double kP = 2.9;
        public static final double kI = 0;
        public static final double kD = 0.50318; //1.7543;
        public static final double kF = 0;
        public static final double kS = 0.1181;
        //public static final double kG = 0.32446;
        public static final double kG = 0.4;
        public static final double kV = 3.4;
        public static final double kA = 0.044465;

        public static final double kSExtended = 0.019459;
        public static final double kGExtended = 0.68405;
        public static final double kVExtended = 3.722;
        public static final double kAExtended = 0.088359;
    }

    public static final class RotationConstraints{
        public static final double MAX_ROTATION_VELOCITY_RPS = 3 * Math.PI / 2;
        public static final double MAX_ROTATION_ACCELERATION_RPSPS = MAX_ROTATION_VELOCITY_RPS * 3;
    }

    public static final class ExtensionSetpoints {
        //
        public static final double LOW = 0.05;
        public static final double MID = 0.3;
        public static final double HIGH = 0.84;
        public static final double DOUBLE_SUBSTATION = 0.2;



    }

    public static final class RotationSetpoints {
        //in degrees initially, conv to rad
        public static final double LOW_RADIANS = Units.degreesToRadians(339.1);
        public static final double MID_RADIANS = Units.degreesToRadians(53);
        public static final double HIGH_RADIANS = Units.degreesToRadians(53);
        public static final double DOUBLE_SUBSTATION_RADIANS = Units.degreesToRadians(69.5);

        public static final double GROUND_CONE_RADIANS = Units.degreesToRadians(342.1);
    }
    //degrees to radians

    //substation height 3 foot 7

    //249 wrist angle

    //317 wrist angle

}
