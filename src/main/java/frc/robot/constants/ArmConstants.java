// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmConstants {
    double placeholder = 0;

    public static final class extensionGains {
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double kF = 0;
    }

    public static final class rotationGains {
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double kF = 0;
    }

    public static final class extensionSetpoints {
        public static double lowNode = 0;
        public static double midNode = 0;
        public static double highNode = 0;


    }

    public static final class angleSetpoints {
        public static double lowNode = 0;
        public static double midNode = 0;
        public static double highNode = 0;
    }

    double intakeTolerance = Units.degreesToRadians(3);
    //degrees to radians


}
