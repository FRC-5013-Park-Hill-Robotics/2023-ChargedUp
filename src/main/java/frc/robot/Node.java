// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class Node {
    private Pose2d lineupPose;
    private Pose2d finishPose;

    public Node(Pose2d lineupPose, Pose2d finishPose) {
        this.lineupPose = lineupPose;
        this.finishPose = finishPose;
    }

    public Pose2d getLineupPose() {
        return this.lineupPose;
    }

    public Pose2d getFinishPose() {
        return this.finishPose;
    }

}
