// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmConstants {
    public static final double ARM_OFFSET_DEGREES = 0;
    public static final class ExtensionGains {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
    }

    public static final class RotationGains {
        public static final Rotation2d TOLERANCE= Rotation2d.fromDegrees(3);
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
    }

    public static final class RotationConstraints{
        public static final double MAX_ROTATION_VELOCITY_RPS = 8.38;
        public static final double MAX_ROTATION_ACCELERATION_RPSPS = MAX_ROTATION_VELOCITY_RPS * 3;
    }

    public static final class ExtensionSetpoints {
        public static final double LOW = 0;
        public static final double MID = 0;
        public static final double HIGH = 0;


    }

    public static final class RotationSetpoints {
        public static final double LOW = 0;
        public static final double MID = 0;
        public static final double HIGH = 0;
    }
    //degrees to radians


}
