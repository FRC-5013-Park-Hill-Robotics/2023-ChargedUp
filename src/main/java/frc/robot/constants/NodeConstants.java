// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;
import org.apache.commons.lang3.ObjectUtils.Null;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Grid;
import frc.robot.Node;


/** Add your docs here. */
public class NodeConstants {
    Alliance alliance = DriverStation.getAlliance();
    //Blue Poses of nodes from grids 1-3
    Pose2d blueLeftLineupPoseLeft = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(1, alliance), null);
    Pose2d blueLeftFinalPoseLeft = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(1, alliance), null);
    Pose2d blueLeftLineupPoseMiddle = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(2, alliance), null);
    Pose2d blueLeftFinalPoseMiddle = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(2, alliance), null);
    Pose2d blueLeftLineupPoseRight = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(3, alliance), null);
    Pose2d blueLeftFinalPoseRight = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(3, alliance), null);

    Pose2d blueMiddleLineupPoseLeft = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(4, alliance), null);
    Pose2d blueMiddleFinalPoseLeft = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(4, alliance), null);
    Pose2d blueMiddleLineupPoseMiddle = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(5, alliance), null);
    Pose2d blueMiddleFinalPoseMiddle = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(5, alliance), null);
    Pose2d blueMiddleLineupPoseRight = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(6, alliance), null);
    Pose2d blueMiddleFinalPoseRight = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(6, alliance), null);

    Pose2d blueRightLineupPoseLeft = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(7, alliance), null);
    Pose2d blueRightFinalPoseLeft = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(7, alliance), null);
    Pose2d blueRightLineupPoseMiddle = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(8, alliance), null);
    Pose2d blueRightFinalPoseMiddle = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(8, alliance), null);
    Pose2d blueRightLineupPoseRight = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(9, alliance), null);
    Pose2d blueRightFinalPoseRight = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(9, alliance), null);

    //Red Poses of nodes from grids 1-3
    Pose2d redLeftLineupPoseLeft = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(1, alliance), null);
    Pose2d redLeftFinalPoseLeft = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(1, alliance), null);
    Pose2d redLeftLineupPoseMiddle = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(2, alliance), null);
    Pose2d redLeftFinalPoseMiddle = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(2, alliance), null);
    Pose2d redLeftLineupPoseRight = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(3, alliance), null);
    Pose2d redLeftFinalPoseRight = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(3, alliance), null);

    Pose2d redMiddleLineupPoseLeft = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(4, alliance), null);
    Pose2d redMiddleFinalPoseLeft = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(4, alliance), null);
    Pose2d redMiddleLineupPoseMiddle = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(5, alliance), null);
    Pose2d redMiddleFinalPoseMiddle = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(5, alliance), null);
    Pose2d redMiddleLineupPoseRight = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(6, alliance), null);
    Pose2d redMiddleFinalPoseRight = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(6, alliance), null);

    Pose2d redRightLineupPoseLeft = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(7, alliance), null);
    Pose2d redRightFinalPoseLeft = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(7, alliance), null);
    Pose2d redRightLineupPoseMiddle = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(8, alliance), null);
    Pose2d redRightFinalPoseMiddle = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(8, alliance), null);
    Pose2d redRightLineupPoseRight = new Pose2d(0.86195, FieldTrajectoryConstants.transformY(9, alliance), null);
    Pose2d redRightFinalPoseRight = new Pose2d(0.36195, FieldTrajectoryConstants.transformY(9, alliance), null);

    //Grid construction on both sides of the field for on-the-fly path generation
    Grid BlueLeft = new Grid(new Node(blueLeftLineupPoseLeft, blueLeftFinalPoseLeft), new Node(blueLeftLineupPoseMiddle, blueLeftFinalPoseMiddle), new Node(blueLeftLineupPoseRight, blueLeftFinalPoseRight));
    Grid BlueMiddle = new Grid(new Node(blueMiddleLineupPoseLeft, blueMiddleFinalPoseLeft), new Node(blueMiddleLineupPoseMiddle, blueMiddleFinalPoseMiddle), new Node(blueMiddleLineupPoseRight, blueMiddleFinalPoseRight));
    Grid BlueRight = new Grid(new Node(blueRightLineupPoseLeft, blueRightFinalPoseLeft), new Node(blueRightLineupPoseMiddle, blueRightFinalPoseMiddle), new Node(blueRightLineupPoseRight, blueRightFinalPoseRight));

    Grid RedLeft = new Grid(new Node(redLeftLineupPoseLeft, redLeftFinalPoseLeft), new Node(redLeftLineupPoseMiddle, redLeftFinalPoseMiddle), new Node(redLeftLineupPoseRight, redLeftFinalPoseRight));
    Grid RedMiddle = new Grid(new Node(redMiddleLineupPoseLeft, redMiddleFinalPoseLeft), new Node(redMiddleLineupPoseMiddle, redMiddleFinalPoseMiddle), new Node(redMiddleLineupPoseRight, redMiddleFinalPoseRight));
    Grid RedRight = new Grid(new Node(redRightLineupPoseLeft, redRightFinalPoseLeft), new Node(redRightLineupPoseMiddle, redRightFinalPoseMiddle), new Node(redRightLineupPoseRight, redRightFinalPoseRight));

    public Grid findGrid(Pose2d pose, Alliance alliance) {
        if ((pose.getY() >= 0) && (pose.getY() <= 1.91) && (Alliance.Blue == alliance)) {
            return BlueLeft;
        }
        else if ((pose.getY() >= 1.91) && (pose.getY() <= 3.59) && (Alliance.Blue == alliance)) {
            return BlueMiddle;
        }
        else if ((pose.getY() >= 3.59) && (pose.getY() <= 5.5) && (Alliance.Blue == alliance)) {
            return BlueRight;
        }

        else if ((pose.getY() >= 0) && (pose.getY() <= 1.91) && (Alliance.Red == alliance)) {
            return RedLeft;
        }
        else if ((pose.getY() >= 1.91) && (pose.getY() <= 3.59) && (Alliance.Red == alliance)) {
            return RedMiddle;
        }
        else if ((pose.getY() >= 3.59) && (pose.getY() <= 5.5) && (Alliance.Red == alliance)) {
            return RedRight;
        }

        else {
            return RedLeft;
        }
        
    }




}
