// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class FieldTrajectoryConstants {
    public static final double fieldWidthMeters = 8.02;

    public static final double robotHalfDistanceMeters = 0.36195;

    public static final double[] yInchesPositionGridArray = {20.19, 42.2475, 64.305, 86.3625, 108.42, 130.4775, 152.535, 174.5925, 196.65};

    public static Alliance allianceColor = DriverStation.getAlliance();

    public static double transformY(int gridIndex, Alliance alliance) {

        if (alliance == Alliance.Blue) {
            double yPositionMeters = Units.inchesToMeters(yInchesPositionGridArray[getIndex(gridIndex)]);
            return yPositionMeters;
        }
        else {
            double yPositionMeters = fieldWidthMeters - Units.inchesToMeters(yInchesPositionGridArray[getIndex(gridIndex)]);
            return yPositionMeters;
        }

    }

    public static int getIndex(int i) {
        return isBlue()?i:(9-i);
    }


    public static boolean isBlue() {
        if (DriverStation.getAlliance() == Alliance.Blue) {
            return true;
        }
        return false;

    }

    //public static Pose2d getPlacement() {
        
    //}
    //public static Pose2d getLineup() {

    //}

}
