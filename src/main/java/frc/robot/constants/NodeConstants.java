// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Grid;
import frc.robot.Node;


/** Add your docs here. */
public class NodeConstants {
    Alliance alliance = DriverStation.getAlliance();
    //Blue Poses of nodes from grids 1-3
    Pose2d blue1LineupPose1 = new Pose2d(0, FieldTrajectoryConstants.transformY(1, alliance), null);
    Pose2d blue1FinalPose1 = new Pose2d(0, FieldTrajectoryConstants.transformY(1, alliance), null);
    Pose2d blue1LineupPose2 = new Pose2d(0, FieldTrajectoryConstants.transformY(2, alliance), null);
    Pose2d blue1FinalPose2 = new Pose2d(0, FieldTrajectoryConstants.transformY(2, alliance), null);
    Pose2d blue1LineupPose3 = new Pose2d(0, FieldTrajectoryConstants.transformY(3, alliance), null);
    Pose2d blue1FinalPose3 = new Pose2d(0, FieldTrajectoryConstants.transformY(3, alliance), null);

    Pose2d blue2LineupPose1 = new Pose2d(0, FieldTrajectoryConstants.transformY(4, alliance), null);
    Pose2d blue2FinalPose1 = new Pose2d(0, FieldTrajectoryConstants.transformY(4, alliance), null);
    Pose2d blue2LineupPose2 = new Pose2d(0, FieldTrajectoryConstants.transformY(5, alliance), null);
    Pose2d blue2FinalPose2 = new Pose2d(0, FieldTrajectoryConstants.transformY(5, alliance), null);
    Pose2d blue2LineupPose3 = new Pose2d(0, FieldTrajectoryConstants.transformY(6, alliance), null);
    Pose2d blue2FinalPose3 = new Pose2d(0, FieldTrajectoryConstants.transformY(6, alliance), null);

    Pose2d blue3LineupPose1 = new Pose2d(0, FieldTrajectoryConstants.transformY(7, alliance), null);
    Pose2d blue3FinalPose1 = new Pose2d(0, FieldTrajectoryConstants.transformY(7, alliance), null);
    Pose2d blue3LineupPose2 = new Pose2d(0, FieldTrajectoryConstants.transformY(8, alliance), null);
    Pose2d blue3FinalPose2 = new Pose2d(0, FieldTrajectoryConstants.transformY(8, alliance), null);
    Pose2d blue3LineupPose3 = new Pose2d(0, FieldTrajectoryConstants.transformY(9, alliance), null);
    Pose2d blue3FinalPose3 = new Pose2d(0, FieldTrajectoryConstants.transformY(9, alliance), null);

    //Red Poses of nodes from grids 1-3
    //TODO: Switch order of poses
    Pose2d red1LineupPose1 = new Pose2d(0, FieldTrajectoryConstants.transformY(1, alliance), null);
    Pose2d red1FinalPose1 = new Pose2d(0, FieldTrajectoryConstants.transformY(1, alliance), null);
    Pose2d red1LineupPose2 = new Pose2d(0, FieldTrajectoryConstants.transformY(2, alliance), null);
    Pose2d red1FinalPose2 = new Pose2d(0, FieldTrajectoryConstants.transformY(2, alliance), null);
    Pose2d red1LineupPose3 = new Pose2d(0, FieldTrajectoryConstants.transformY(3, alliance), null);
    Pose2d red1FinalPose3 = new Pose2d(0, FieldTrajectoryConstants.transformY(3, alliance), null);

    Pose2d red2LineupPose1 = new Pose2d(0, FieldTrajectoryConstants.transformY(4, alliance), null);
    Pose2d red2FinalPose1 = new Pose2d(0, FieldTrajectoryConstants.transformY(4, alliance), null);
    Pose2d red2LineupPose2 = new Pose2d(0, FieldTrajectoryConstants.transformY(5, alliance), null);
    Pose2d red2FinalPose2 = new Pose2d(0, FieldTrajectoryConstants.transformY(5, alliance), null);
    Pose2d red2LineupPose3 = new Pose2d(0, FieldTrajectoryConstants.transformY(6, alliance), null);
    Pose2d red2FinalPose3 = new Pose2d(0, FieldTrajectoryConstants.transformY(6, alliance), null);

    Pose2d red3LineupPose1 = new Pose2d(0, FieldTrajectoryConstants.transformY(7, alliance), null);
    Pose2d red3FinalPose1 = new Pose2d(0, FieldTrajectoryConstants.transformY(7, alliance), null);
    Pose2d red3LineupPose2 = new Pose2d(0, FieldTrajectoryConstants.transformY(8, alliance), null);
    Pose2d red3FinalPose2 = new Pose2d(0, FieldTrajectoryConstants.transformY(8, alliance), null);
    Pose2d red3LineupPose3 = new Pose2d(0, FieldTrajectoryConstants.transformY(9, alliance), null);
    Pose2d red3FinalPose3 = new Pose2d(0, FieldTrajectoryConstants.transformY(9, alliance), null);

    //Grid construction on both sides of the field for on-the-fly path gen
    Grid BlueLeft = new Grid(new Node(blue1LineupPose1, blue1FinalPose1), new Node(blue1LineupPose2, blue1FinalPose2), new Node(blue1LineupPose3, blue1FinalPose3));
    Grid BlueMiddle = new Grid(new Node(blue2LineupPose1, blue2FinalPose1), new Node(blue2LineupPose2, blue2FinalPose2), new Node(blue2LineupPose3, blue2FinalPose3));
    Grid BlueRight = new Grid(new Node(blue3LineupPose1, blue3FinalPose1), new Node(blue3LineupPose2, blue3FinalPose2), new Node(blue3LineupPose3, blue3FinalPose3));

    Grid RedLeft = new Grid(new Node(red1LineupPose1, red1FinalPose1), new Node(red1LineupPose2, red1FinalPose2), new Node(red1LineupPose3, red1FinalPose3));
    Grid RedMiddle = new Grid(new Node(red2LineupPose1, red2FinalPose1), new Node(red2LineupPose2, red2FinalPose2), new Node(red2LineupPose3, red2FinalPose3));
    Grid RedRight = new Grid(new Node(red3LineupPose1, red3FinalPose1), new Node(red3LineupPose2, red3FinalPose2), new Node(red3LineupPose3, red3FinalPose3));



}
