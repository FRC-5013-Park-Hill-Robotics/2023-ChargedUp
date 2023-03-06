// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Node;
import frc.robot.Grid;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldTrajectoryConstants;
import frc.robot.constants.NodeConstants;

public class DriveToLocation extends CommandBase {
  private Drivetrain m_drivetrain;
  /** Creates a new DriveToLocation. */
  public DriveToLocation(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentYMeters = m_drivetrain.getPose().getTranslation().getY();
    Alliance alliance = DriverStation.getAlliance();
    Grid currentGrid = NodeConstants.findGrid(currentYMeters, alliance);
    Node currentNode = NodeConstants.findNode(currentYMeters, currentGrid);
    Pose2d nodeLineupPose2d = currentNode.getLineupPose();
    Pose2d nodeFinalPose2d = currentNode.getFinishPose();
    //PathPlannerTrajectory traj1 = PathPlanner.generatePath(
    //new PathConstraints(4, 3), 
    //new PathPoint(nodeLineupPose2d, Rotation2d.fromDegrees(0)), // position, heading
    //new PathPoint(nodeFinalPose2d, Rotation2d.fromDegrees(45)) // position, heading
    //);

    //needs the current y pose and alliance
    //determines the grid using the pose and alliance
    //determines the node using the pose and alliance
    //creates a path that drives to the desired lineup pose
    //then the path should drive to the desired final pose
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
