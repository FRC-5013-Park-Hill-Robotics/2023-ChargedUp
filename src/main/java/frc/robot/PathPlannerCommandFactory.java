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
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.constants.DrivetrainConstants;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.DrivetrainConstants.ThetaGains;
import frc.robot.constants.DrivetrainConstants.TranslationGains;
import frc.robot.subsystems.Drivetrain;
import frc.robot.trobot5013lib.command.TrajectoryLogging;
import frc.robot.AutonomousCommandFactory;
import frc.robot.RobotContainer;
import frc.robot.Robot;

/** Add your docs here. */
public class PathPlannerCommandFactory {
    public static PathPlannerTrajectory trajectory2HAB;
    //public static List<PathPlannerTrajectory> trajectory?; 
    public static void init() {
        trajectory2HAB = PathPlanner.loadPath("2HAB",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        //trajectory? = PathPlanner.loadPathGroup("?", MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33 )
    }

    public static double MAX_AUTO_VELOCITY_METERS_PER_SECOND = DrivetrainConstants.maxSpeed/2;
	public static double MAX_AUTO_ANGULAR_RADIANS_PER_SECOND = DrivetrainConstants.maxAngularVelocity/2;
	public static String path2HAB = "2HAB";
	public static String pathCurvy = "Curvy";

	public static final String[] AUTOS = {path2HAB, pathCurvy};

    public static Command createAutonomous(RobotContainer container, String name) {
		if (path2HAB.equals(name)) {
			return create2HAB(container);
		}
		else {
			return create2HAB(container);
		}
	}
    /// call it inside the methods, instead of declaring in main space SwerveAutoBuilder autoBuilder = getSwerveAutoBuilder();

    public static Command create2HAB(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2HAB", new PathConstraints(3, 4));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());
        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    //List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3)); 
    //SwerveAutoBuilder m_autoBuilder = getSwerveAutoBuilder();
    //Command fullAuto = m_autoBuilder.fullAuto(pathGroup);


}

