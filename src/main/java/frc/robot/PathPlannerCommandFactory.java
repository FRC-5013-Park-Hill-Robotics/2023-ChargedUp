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
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.ArmConstants.ExtensionSetpoints;
import frc.robot.constants.ArmConstants.RotationSetpoints;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.DrivetrainConstants.ThetaGains;
import frc.robot.constants.DrivetrainConstants.TranslationGains;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.trobot5013lib.command.TrajectoryLogging;
import frc.robot.AutonomousCommandFactory;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmExtend;
import frc.robot.commands.ArmRotate;
import frc.robot.Robot;

/** Add your docs here. */
public class PathPlannerCommandFactory {
    public static PathPlannerTrajectory trajectory1MA;
    public static PathPlannerTrajectory trajectory2A;
    public static PathPlannerTrajectory trajectory3A;
    public static PathPlannerTrajectory trajectory4A;
    public static PathPlannerTrajectory trajectory5A;
    public static PathPlannerTrajectory trajectory5U;
    public static PathPlannerTrajectory trajectory6U;
    public static PathPlannerTrajectory trajectory7U;
    public static PathPlannerTrajectory trajectory8U;
    public static PathPlannerTrajectory trajectory9U;

    public static PathPlannerTrajectory trajectory2HAB;
    public static PathPlannerTrajectory trajectoryKittyWhipper;
    public static PathPlannerTrajectory trajectoryFirstGrid;


    //public static List<PathPlannerTrajectory> trajectory?; 
    public static void init() {
        trajectory1MA = PathPlanner.loadPath("1MA",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        trajectory2A = PathPlanner.loadPath("2A",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        trajectory3A = PathPlanner.loadPath("3A",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        trajectory4A = PathPlanner.loadPath("4A",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        trajectory5A = PathPlanner.loadPath("5A",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        trajectory5U = PathPlanner.loadPath("5U",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        trajectory6U = PathPlanner.loadPath("6U",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        trajectory7U = PathPlanner.loadPath("7U",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        trajectory8U = PathPlanner.loadPath("8U",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        trajectory9U = PathPlanner.loadPath("9U",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);

        trajectoryFirstGrid = PathPlanner.generatePath(
            new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_VELOCITY_METERS_PER_SECOND/.33),
            new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90)), // position, heading(direction of travel), holonomic rotation
            new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(90))); // position, heading(direction of travel), holonomic rotation

        //trajectory? = PathPlanner.loadPathGroup("?", MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33 )
    }

    public static double MAX_AUTO_VELOCITY_METERS_PER_SECOND = DrivetrainConstants.maxSpeed/2;
	public static double MAX_AUTO_ANGULAR_RADIANS_PER_SECOND = DrivetrainConstants.maxAngularVelocity/2;
    public static String path1MA = "1MA";
    public static String path2A = "2A";
    public static String path3A = "3A";
    public static String path4A = "4A";
    public static String path5A = "5A";
    public static String path5U = "5U";
    public static String path6U = "6U";
    public static String path7U = "7U";
    public static String path8U = "8U";
    public static String path9U = "9U";

	public static String path2HAB = "2HAB";
	public static String pathCurvy = "Curvy";
    public static String pathKittyWhipper = "KittyWhipper";

	public static final String[] AUTOS = {path2HAB, pathCurvy, pathKittyWhipper, path1MA, path2A, path3A, path4A, path5A, path5U, path6U, path7U, path8U, path9U};

    public static Command createAutonomous(RobotContainer container, String name) {
		if (path2HAB.equals(name)) {
			return create2HAB(container);
		}

        else if (path1MA.equals(name)) {
            return create1MA(container);
        }

        else if (path2A.equals(name)) {
            return create2A(container);
        }

        else if (path3A.equals(name)) {
            return create3A(container);
        }

        else if (path4A.equals(name)) {
            return create4A(container);
        }

        else if (path5A.equals(name)) {
            return create5A(container);
        }

        else if (path5U.equals(name)) {
            return create5U(container);
        }

        else if (path6U.equals(name)) {
            return create6U(container);
        }

        else if (path7U.equals(name)) {
            return create7U(container);
        }

        else if (path8U.equals(name)) {
            return create8U(container);
        }

        else if (path9U.equals(name)) {
            return create9U(container);
        }

        else if (pathKittyWhipper.equals(name)) {
            return createKittyWhipper(container);
        }
		else {
			return create2HAB(container);
		}
	}
    /// call it inside the methods, instead of declaring in main space SwerveAutoBuilder autoBuilder = getSwerveAutoBuilder();

    private static Command create1MA(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("1MA", new PathConstraints(3, 4));
        HashMap<String, Command> eventMap = new HashMap<>();
        SequentialCommandGroup mid = new SequentialCommandGroup(new ArmRotate(container.getArm(), 0),
            new ArmExtend(container.getArm(), ExtensionSetpoints.MID),
            new ArmRotate(container.getArm(), RotationSetpoints.MID_RADIANS));
        SequentialCommandGroup dropCone = new SequentialCommandGroup(new InstantCommand(container.getIntake()::pickUpCube),
            new ArmRotate(container.getArm(), RotationSetpoints.MID_RADIANS));
        eventMap.put("Mid", mid);
        eventMap.put("Drop Cone", dropCone);
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    private static Command create2A(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2A", new PathConstraints(3, 4));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    private static Command create3A(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("3A", new PathConstraints(3, 4));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    private static Command create4A(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("4A", new PathConstraints(3, 4));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    private static Command create5A(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("5A", new PathConstraints(3, 4));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    private static Command create5U(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("5U", new PathConstraints(3, 4));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    private static Command create6U(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("6U", new PathConstraints(3, 4));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    private static Command create7U(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("7U", new PathConstraints(3, 4));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    private static Command create8U(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("8U", new PathConstraints(3, 4));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    private static Command create9U(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("9U", new PathConstraints(3, 4));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    public static Command create2HAB(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2HAB", new PathConstraints(3, 4));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    public static Command createKittyWhipper(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("KittyWhipper", new PathConstraints(3, 4));
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

