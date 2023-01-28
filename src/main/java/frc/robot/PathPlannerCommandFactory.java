// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.constants.DrivetrainConstants;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.DrivetrainConstants.DrivetrainGeometry;
import frc.robot.constants.DrivetrainConstants.ThetaGains;
import frc.robot.constants.DrivetrainConstants.TranslationGains;
import frc.robot.subsystems.Drivetrain;
import frc.robot.trobot5013lib.command.TrajectoryLogging;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.AutonomousCommandFactory;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class PathPlannerCommandFactory {
    /// call it inside the methods, instead of declaring in main space SwerveAutoBuilder autoBuilder = getSwerveAutoBuilder();

    //List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3)); 
    //SwerveAutoBuilder m_autoBuilder = getSwerveAutoBuilder();
    //Command fullAuto = m_autoBuilder.fullAuto(pathGroup);


}

