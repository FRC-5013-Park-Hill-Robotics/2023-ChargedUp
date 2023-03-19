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
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.Robot;

/** Add your docs here. */
public class PathPlannerCommandFactory {
    public static PathPlannerTrajectory trajectoryLeftMid;
    public static PathPlannerTrajectory trajectoryRightMid;
    public static PathPlannerTrajectory trajectoryEngage;
    public static PathPlannerTrajectory trajectoryStraight;

    public static PathPlannerTrajectory trajectoryMidPlace;
    public static PathPlannerTrajectory trajectoryMiddleMidEngage;

    public static PathPlannerTrajectory trajectory2HAB;
    public static PathPlannerTrajectory trajectoryFirstGrid;


    //public static List<PathPlannerTrajectory> trajectory?; 
    public static void init() {
        trajectory2HAB = PathPlanner.loadPath("2HAB",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        trajectoryLeftMid = PathPlanner.loadPath("LeftMid",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        trajectoryRightMid = PathPlanner.loadPath("RightMid",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        trajectoryEngage = PathPlanner.loadPath("Engage",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        trajectoryStraight = PathPlanner.loadPath("Straight",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        trajectoryMidPlace = PathPlanner.loadPath("Mid Place",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);
        trajectoryMiddleMidEngage = PathPlanner.loadPath("MiddleMidEngage",
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
    public static String pathLeftMid = "LeftMid";
    public static String pathRightMid = "RightMid";
    public static String pathEngage = "Engage";
    public static String pathStraight = "Straight";
    public static String pathMidPlace = "Mid Place";
    public static String pathMiddleMidEngage = "MiddleMidEngage";

	public static String path2HAB = "2HAB";

	public static final String[] AUTOS = {path2HAB, pathLeftMid, pathEngage, pathStraight, pathRightMid, pathMidPlace, pathMiddleMidEngage};

    public static Command createAutonomous(RobotContainer container, String name) {
		if (path2HAB.equals(name)) {
			return create2HAB(container);
		}

        else if (pathLeftMid.equals(name)) {
            return createLeftMid(container);
        }

        else if (pathEngage.equals(name)) {
            return createEngage(container);
        }

        else if (pathStraight.equals(name)) {
            return createStraight(container);
        }

        else if (pathRightMid.equals(name)) {
            return createRightMid(container);
        }

        else if (pathMidPlace.equals(name)) {
            return createMidPlace(container);
        }

        else if (pathMiddleMidEngage.equals(name)) {
            return createMiddleMidEngage(container);
        }

		else {
			return create2HAB(container);
		}
	}
    /// call it inside the methods, instead of declaring in main space SwerveAutoBuilder autoBuilder = getSwerveAutoBuilder();

    private static Command createLeftMid(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("1MA", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_VELOCITY_METERS_PER_SECOND/3));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
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

    private static Command createRightMid(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("RightMid", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_VELOCITY_METERS_PER_SECOND/3));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
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

    private static Command createMidPlace(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Mid Place", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_VELOCITY_METERS_PER_SECOND/3));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
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

    private static Command createEngage(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Engage", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_VELOCITY_METERS_PER_SECOND/3));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
        //eventMap.put("Balance", new AutoBalanceCommand(container.getDrivetrain()));
        //eventMap.put("X", new RunCommand(container.getDrivetrain()::setX));
        //eventMap.put("intakeDown", new IntakeDown());

        Command path = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        Command fullAuto = path.andThen( new AutoBalanceCommand(container.getDrivetrain()));            
         
        return fullAuto;
    }

    public static Command create2HAB(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2HAB", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_VELOCITY_METERS_PER_SECOND/3));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    private static Command createStraight(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Straight", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_VELOCITY_METERS_PER_SECOND/3));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    private static Command createMiddleMidEngage(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("MiddleMidEngage", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_VELOCITY_METERS_PER_SECOND/3));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
        //eventMap.put("Balance", new AutoBalanceCommand(container.getDrivetrain()));
        //eventMap.put("X", new RunCommand(container.getDrivetrain()::setX));
        //eventMap.put("intakeDown", new IntakeDown());
        SequentialCommandGroup mid = new SequentialCommandGroup(new ArmRotate(container.getArm(), 0),
            new ArmExtend(container.getArm(), ExtensionSetpoints.MID),
            new ArmRotate(container.getArm(), RotationSetpoints.MID_RADIANS));
        SequentialCommandGroup dropCone = new SequentialCommandGroup(new InstantCommand(container.getIntake()::pickUpCube),
            new ArmRotate(container.getArm(), RotationSetpoints.MID_RADIANS));

        eventMap.put("Mid", mid);
        eventMap.put("Drop Cone", dropCone);

        Command path = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        Command fullAuto = path.andThen( new AutoBalanceCommand(container.getDrivetrain()));            
         
        return fullAuto;
    }

    //List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3)); 
    //SwerveAutoBuilder m_autoBuilder = getSwerveAutoBuilder();
    //Command fullAuto = m_autoBuilder.fullAuto(pathGroup);


}

