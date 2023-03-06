// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Grid;
import frc.robot.Node;


/** Add your docs here. */
public class NodeConstants {
    public static final Alliance alliance = DriverStation.getAlliance();
    //Blue Poses of nodes from grids 1-3
    public static final Pose2d blueLeftLineupPoseLeft = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(1, alliance), null);
    public static final Pose2d blueLeftFinalPoseLeft = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(1, alliance), null);
    public static final Pose2d blueLeftLineupPoseMiddle = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(2, alliance), null);
    public static final Pose2d blueLeftFinalPoseMiddle = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(2, alliance), null);
    public static final Pose2d blueLeftLineupPoseRight = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(3, alliance), null);
    public static final Pose2d blueLeftFinalPoseRight = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(3, alliance), null);

    public static final Pose2d blueMiddleLineupPoseLeft = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(4, alliance), null);
    public static final Pose2d blueMiddleFinalPoseLeft = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(4, alliance), null);
    public static final Pose2d blueMiddleLineupPoseMiddle = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(5, alliance), null);
    public static final Pose2d blueMiddleFinalPoseMiddle = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(5, alliance), null);
    public static final Pose2d blueMiddleLineupPoseRight = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(6, alliance), null);
    public static final Pose2d blueMiddleFinalPoseRight = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(6, alliance), null);

    public static final Pose2d blueRightLineupPoseLeft = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(7, alliance), null);
    public static final Pose2d blueRightFinalPoseLeft = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(7, alliance), null);
    public static final Pose2d blueRightLineupPoseMiddle = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(8, alliance), null);
    public static final Pose2d blueRightFinalPoseMiddle = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(8, alliance), null);
    public static final Pose2d blueRightLineupPoseRight = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(9, alliance), null);
    public static final Pose2d blueRightFinalPoseRight = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(9, alliance), null);

    //Red Poses of nodes from grids 1-3
    public static final Pose2d redLeftLineupPoseLeft = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(1, alliance), null);
    public static final Pose2d redLeftFinalPoseLeft = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(1, alliance), null);
    public static final Pose2d redLeftLineupPoseMiddle = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(2, alliance), null);
    public static final Pose2d redLeftFinalPoseMiddle = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(2, alliance), null);
    public static final Pose2d redLeftLineupPoseRight = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(3, alliance), null);
    public static final Pose2d redLeftFinalPoseRight = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(3, alliance), null);

    public static final Pose2d redMiddleLineupPoseLeft = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(4, alliance), null);
    public static final Pose2d redMiddleFinalPoseLeft = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(4, alliance), null);
    public static final Pose2d redMiddleLineupPoseMiddle = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(5, alliance), null);
    public static final Pose2d redMiddleFinalPoseMiddle = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(5, alliance), null);
    public static final Pose2d redMiddleLineupPoseRight = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(6, alliance), null);
    public static final Pose2d redMiddleFinalPoseRight = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(6, alliance), null);

    public static final Pose2d redRightLineupPoseLeft = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(7, alliance), null);
    public static final Pose2d redRightFinalPoseLeft = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(7, alliance), null);
    public static final Pose2d redRightLineupPoseMiddle = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(8, alliance), null);
    public static final Pose2d redRightFinalPoseMiddle = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(8, alliance), null);
    public static final Pose2d redRightLineupPoseRight = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(9, alliance), null);
    public static final Pose2d redRightFinalPoseRight = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(9, alliance), null);

    //Grid construction on both sides of the field for on-the-fly path generation
    public static final Grid BlueLeft = new Grid(new Node(blueLeftLineupPoseLeft, blueLeftFinalPoseLeft), new Node(blueLeftLineupPoseMiddle, blueLeftFinalPoseMiddle), new Node(blueLeftLineupPoseRight, blueLeftFinalPoseRight));
    public static final Grid BlueMiddle = new Grid(new Node(blueMiddleLineupPoseLeft, blueMiddleFinalPoseLeft), new Node(blueMiddleLineupPoseMiddle, blueMiddleFinalPoseMiddle), new Node(blueMiddleLineupPoseRight, blueMiddleFinalPoseRight));
    public static final Grid BlueRight = new Grid(new Node(blueRightLineupPoseLeft, blueRightFinalPoseLeft), new Node(blueRightLineupPoseMiddle, blueRightFinalPoseMiddle), new Node(blueRightLineupPoseRight, blueRightFinalPoseRight));

    public static final Grid RedLeft = new Grid(new Node(redLeftLineupPoseLeft, redLeftFinalPoseLeft), new Node(redLeftLineupPoseMiddle, redLeftFinalPoseMiddle), new Node(redLeftLineupPoseRight, redLeftFinalPoseRight));
    public static final Grid RedMiddle = new Grid(new Node(redMiddleLineupPoseLeft, redMiddleFinalPoseLeft), new Node(redMiddleLineupPoseMiddle, redMiddleFinalPoseMiddle), new Node(redMiddleLineupPoseRight, redMiddleFinalPoseRight));
    public static final Grid RedRight = new Grid(new Node(redRightLineupPoseLeft, redRightFinalPoseLeft), new Node(redRightLineupPoseMiddle, redRightFinalPoseMiddle), new Node(redRightLineupPoseRight, redRightFinalPoseRight));


    // Blue Node edges (meaning the start y position of the node)
    public static final double blueLeftEdgeYLeft = 0;
    public static final double blueLeftEdgeYMiddle = Units.inchesToMeters(25.75);
    public static final double blueLeftEdgeYRight = Units.inchesToMeters(44.25);

    public static final double blueMiddleEdgeYLeft = Units.inchesToMeters(62.75);
    public static final double blueMiddleEdgeYMiddle = Units.inchesToMeters(81.25);
    public static final double blueMiddleEdgeYRight = Units.inchesToMeters(99.75);

    public static final double blueRightEdgeYLeft = Units.inchesToMeters(118.25);
    public static final double blueRightEdgeYMiddle = Units.inchesToMeters(136.75);
    public static final double blueRightEdgeYRight = Units.inchesToMeters(155.25);

    public static final double blueFurthestRightEdge = Units.inchesToMeters(181);

    // Red Node edges (meaning the start y position of the node)
    public static final double redLeftEdgeYLeft = Units.inchesToMeters(FieldTrajectoryConstants.fieldWidthInches);
    public static final double redLeftEdgeYMiddle = Units.inchesToMeters(FieldTrajectoryConstants.fieldWidthInches-25.75);
    public static final double redLeftEdgeYRight = Units.inchesToMeters(FieldTrajectoryConstants.fieldWidthInches-44.25);

    public static final double redMiddleEdgeYLeft = Units.inchesToMeters(FieldTrajectoryConstants.fieldWidthInches-62.75);
    public static final double redMiddleEdgeYMiddle = Units.inchesToMeters(FieldTrajectoryConstants.fieldWidthInches-81.25);
    public static final double redMiddleEdgeYRight = Units.inchesToMeters(FieldTrajectoryConstants.fieldWidthInches-99.75);

    public static final double redRightEdgeYLeft = Units.inchesToMeters(FieldTrajectoryConstants.fieldWidthInches-118.25);
    public static final double redRightEdgeYMiddle = Units.inchesToMeters(FieldTrajectoryConstants.fieldWidthInches-136.75);
    public static final double redRightEdgeYRight = Units.inchesToMeters(FieldTrajectoryConstants.fieldWidthInches-155.25);

    public static final double redFurthestRightEdge = Units.inchesToMeters(FieldTrajectoryConstants.fieldWidthInches-181);

    public static Grid findGrid(double currentY, Alliance alliance) {
        if ((currentY >= 0) && (currentY <= 1.91) && (Alliance.Blue == alliance)) {
            return BlueLeft;
        }
        else if ((currentY >= 1.91) && (currentY <= 3.59) && (Alliance.Blue == alliance)) {
            return BlueMiddle;
        }
        else if ((currentY >= 3.59) && (currentY <= 5.5) && (Alliance.Blue == alliance)) {
            return BlueRight;
        }

        else if ((FieldTrajectoryConstants.fieldWidthMeters-currentY >= 0) && (FieldTrajectoryConstants.fieldWidthMeters-currentY <= 1.91) && (Alliance.Red == alliance)) {
            return RedLeft;
        }
        else if ((FieldTrajectoryConstants.fieldWidthMeters-currentY >= 1.91) && (FieldTrajectoryConstants.fieldWidthMeters-currentY <= 3.59) && (Alliance.Red == alliance)) {
            return RedMiddle;
        }
        else if ((FieldTrajectoryConstants.fieldWidthMeters-currentY >= 3.59) && (FieldTrajectoryConstants.fieldWidthMeters-currentY <= 5.5) && (Alliance.Red == alliance)) {
            return RedRight;
        }

        else {
            return null;
        }
        
    }

    public static Node findNode(double currentYMeters, Grid currentGrid) {
        //blue node finding
        if ((currentGrid.equals(BlueLeft) && (blueLeftEdgeYLeft < currentYMeters) && (blueLeftEdgeYMiddle > currentYMeters)))  {
            return BlueLeft.getLeftNode();
        }
        if ((currentGrid.equals(BlueLeft) && (blueLeftEdgeYMiddle < currentYMeters) && (blueLeftEdgeYRight > currentYMeters)))  {
            return BlueLeft.getMiddleNode();
        }
        if ((currentGrid.equals(BlueLeft) && (blueLeftEdgeYRight < currentYMeters) && (blueMiddleEdgeYLeft > currentYMeters)))  {
            return BlueLeft.getRightNode();
        }
        if ((currentGrid.equals(BlueMiddle) && (blueMiddleEdgeYLeft < currentYMeters) && (blueMiddleEdgeYMiddle > currentYMeters)))  {
            return BlueLeft.getLeftNode();
        }
        if ((currentGrid.equals(BlueMiddle) && (blueMiddleEdgeYMiddle < currentYMeters) && (blueMiddleEdgeYRight > currentYMeters)))  {
            return BlueLeft.getMiddleNode();
        }
        if ((currentGrid.equals(BlueMiddle) && (blueMiddleEdgeYRight < currentYMeters) && (blueRightEdgeYLeft > currentYMeters)))  {
            return BlueLeft.getRightNode();
        }
        if ((currentGrid.equals(BlueRight) && (blueRightEdgeYLeft < currentYMeters) && (blueRightEdgeYMiddle > currentYMeters)))  {
            return BlueLeft.getLeftNode();
        }
        if ((currentGrid.equals(BlueRight) && (blueRightEdgeYMiddle < currentYMeters) && (blueRightEdgeYRight > currentYMeters)))  {
            return BlueLeft.getMiddleNode();
        }
        if ((currentGrid.equals(BlueRight) && (blueRightEdgeYRight < currentYMeters) && (blueFurthestRightEdge > currentYMeters)))  {
            return BlueLeft.getRightNode();
        }
        
        //red node finding

        if ((currentGrid.equals(RedLeft) && (redLeftEdgeYLeft < currentYMeters) && (redLeftEdgeYMiddle > currentYMeters)))  {
            return RedLeft.getLeftNode();
        }
        if ((currentGrid.equals(RedLeft) && (redLeftEdgeYMiddle < currentYMeters) && (redLeftEdgeYRight > currentYMeters)))  {
            return RedLeft.getMiddleNode();
        }
        if ((currentGrid.equals(RedLeft) && (redLeftEdgeYRight < currentYMeters) && (redMiddleEdgeYLeft > currentYMeters)))  {
            return RedLeft.getRightNode();
        }
        if ((currentGrid.equals(RedMiddle) && (redMiddleEdgeYLeft < currentYMeters) && (redMiddleEdgeYMiddle > currentYMeters)))  {
            return RedLeft.getLeftNode();
        }
        if ((currentGrid.equals(RedMiddle) && (redMiddleEdgeYMiddle < currentYMeters) && (redMiddleEdgeYRight > currentYMeters)))  {
            return RedLeft.getMiddleNode();
        }
        if ((currentGrid.equals(RedMiddle) && (redMiddleEdgeYRight < currentYMeters) && (redRightEdgeYLeft > currentYMeters)))  {
            return RedLeft.getRightNode();
        }
        if ((currentGrid.equals(RedRight) && (redRightEdgeYLeft < currentYMeters) && (redRightEdgeYMiddle > currentYMeters)))  {
            return RedLeft.getLeftNode();
        }
        if ((currentGrid.equals(RedRight) && (redRightEdgeYMiddle < currentYMeters) && (redRightEdgeYRight > currentYMeters)))  {
            return RedLeft.getMiddleNode();
        }
        if ((currentGrid.equals(RedRight) && (redRightEdgeYRight < currentYMeters) && (redFurthestRightEdge > currentYMeters)))  {
            return RedLeft.getRightNode();
        }

        else {
            return null;
        }
    }



}
