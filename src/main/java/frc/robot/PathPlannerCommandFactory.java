// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmExtend;
import frc.robot.commands.ArmExtendAndRotate;
import frc.robot.commands.ArmRotate;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ArmConstants.ExtensionSetpoints;
import frc.robot.constants.ArmConstants.RotationSetpoints;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

/** Add your docs here. */
public class PathPlannerCommandFactory {
    public static PathPlannerTrajectory trajectoryLeftMid;

    public static PathPlannerTrajectory trajectoryBumperHigh;
    public static PathPlannerTrajectory trajectoryBarrierHigh;
    public static PathPlannerTrajectory trajectoryBarrierHighEngage;
    public static PathPlannerTrajectory trajectoryMiddleHighEngage;
    public static PathPlannerTrajectory trajectoryBarrierHighLeg2;
    public static PathPlannerTrajectory trajectoryBumperHighEngage;

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
		MAX_AUTO_ACCELERATION_METERS_PER_SECOND);
        trajectoryLeftMid = PathPlanner.loadPath("LeftMid",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_ACCELERATION_METERS_PER_SECOND);

        trajectoryBumperHigh = PathPlanner.loadPath("BumperHigh",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_ACCELERATION_METERS_PER_SECOND);
        trajectoryBarrierHigh = PathPlanner.loadPath("BarrierHigh",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_ACCELERATION_METERS_PER_SECOND);
        trajectoryBarrierHighEngage = PathPlanner.loadPath("BarrierHighEngage",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_ACCELERATION_METERS_PER_SECOND);
        trajectoryMiddleHighEngage = PathPlanner.loadPath("MiddleHighEngage",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_ACCELERATION_METERS_PER_SECOND);
        trajectoryBumperHighEngage = PathPlanner.loadPath("BumperHighEngage",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_ACCELERATION_METERS_PER_SECOND);
        
        trajectoryEngage = PathPlanner.loadPath("Engage",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_ACCELERATION_METERS_PER_SECOND);
        trajectoryStraight = PathPlanner.loadPath("Straight",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_ACCELERATION_METERS_PER_SECOND);
        trajectoryMidPlace = PathPlanner.loadPath("Mid Place",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_ACCELERATION_METERS_PER_SECOND);
        trajectoryMiddleMidEngage = PathPlanner.loadPath("MiddleMidEngage",
		MAX_AUTO_VELOCITY_METERS_PER_SECOND,
		MAX_AUTO_ACCELERATION_METERS_PER_SECOND);


        trajectoryFirstGrid = PathPlanner.generatePath(
            new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_VELOCITY_METERS_PER_SECOND/.33),
            new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90)), // position, heading(direction of travel), holonomic rotation
            new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(90))); // position, heading(direction of travel), holonomic rotation

        //trajectory? = PathPlanner.loadPathGroup("?", MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33 )
    }

    public static double MAX_AUTO_VELOCITY_METERS_PER_SECOND = DrivetrainConstants.maxSpeed/2;
    public static double MAX_AUTO_ACCELERATION_METERS_PER_SECOND = MAX_AUTO_VELOCITY_METERS_PER_SECOND;


	public static double MAX_AUTO_ANGULAR_RADIANS_PER_SECOND = DrivetrainConstants.maxAngularVelocity/2;
    public static String pathLeftMid = "LeftMid";

    public static String pathBumperHigh = "BumperHigh";
    public static String pathBarrierHigh = "BarrierHigh";
    public static String pathBarrierHighEngage = "BarrierHighEngage";
    public static String pathMiddleHighEngage = "MiddleHighEngage";
    public static String pathBumperHighEngage = "BumperHighEngage";

    public static String pathEngage = "Engage";
    public static String pathStraight = "Straight";
    public static String pathMidPlace = "Mid Place";
    public static String pathMiddleMidEngage = "MiddleMidEngage";

	public static String path2HAB = "2HAB";

	public static final String[] AUTOS = {path2HAB, pathLeftMid, pathEngage, pathStraight, pathBumperHigh, pathMidPlace, pathMiddleMidEngage, pathBarrierHigh, pathBarrierHighEngage, pathMiddleHighEngage, pathBumperHighEngage};

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

        else if (pathBumperHigh.equals(name)) {
            return createBumperHigh(container);
        }

        else if (pathBarrierHigh.equals(name)) {
            return createBarrierHigh(container);
        }

        else if (pathBarrierHighEngage.equals(name)) {
            return createBarrierHighEngage(container);
        }

        else if (pathMiddleHighEngage.equals(name)) {
            return createMiddleHighEngage(container);
        }

        else if (pathBumperHighEngage.equals(name)) {
            return createBumperHighEngage(container);
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
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("1MA", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_ACCELERATION_METERS_PER_SECOND));
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

    private static Command createBumperHigh(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("BumperHigh", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_ACCELERATION_METERS_PER_SECOND));
        PathPlannerTrajectory path2 = PathPlanner.loadPath("BumperHighLeg2", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_ACCELERATION_METERS_PER_SECOND));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
       // eventMap.put("High", placeConeHigh(container.getArm(), container.getIntake()));
        //eventMap.put("intakeDown", new IntakeDown());

        eventMap.put("Lower Arm", new ArmExtendAndRotate(container.getArm(), ExtensionSetpoints.LOW, RotationSetpoints.LOW_RADIANS));
        eventMap.put("Cone Intake", new RunCommand(container.getIntake()::pickUpCone));

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        Command leg2 = RobotContainer.getSwerveAutoBuilder().followPath(path2);

        return placeConeHigh(container.getArm(), container.getIntake())
        .andThen(fullAuto)
        .andThen(new WaitCommand(0.5))
        .andThen(container.getIntake()::stop)
        .andThen(new ArmExtendAndRotate(container.getArm(),0, Math.PI/2));
        //.alongWith(leg2));
    }

    public static Command resetDrive(Drivetrain drive){
        return new InstantCommand(drive::resetModulesToAbsolute);
    }

    private static Command createBarrierHigh(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("BarrierHigh", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_ACCELERATION_METERS_PER_SECOND));
        PathPlannerTrajectory path2 = PathPlanner.loadPath("BarrierHighLeg2", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_ACCELERATION_METERS_PER_SECOND));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
       // eventMap.put("High", placeConeHigh(container.getArm(), container.getIntake()));
        //eventMap.put("intakeDown", new IntakeDown());

        eventMap.put("Lower Arm", new ArmExtendAndRotate(container.getArm(), ExtensionSetpoints.LOW, RotationSetpoints.GROUND_CONE_RADIANS));
        eventMap.put("Cone Intake", new RunCommand(container.getIntake()::pickUpCone));

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        Command leg2 = RobotContainer.getSwerveAutoBuilder().followPath(path2);
        //Command leg2 = RobotContainer.getSwerveAutoBuilder().fullAuto(trajectory2HAB);
        return placeConeHigh(container.getArm(), container.getIntake())
            .andThen(fullAuto)
            .andThen(new WaitCommand(0.5))
            .andThen(container.getIntake()::stop)
            .andThen(new ArmExtendAndRotate(container.getArm(),0, Math.PI/2)
            .alongWith(leg2));

        
    }



    private static Command createBarrierHighEngage(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("BarrierHighEngage", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_ACCELERATION_METERS_PER_SECOND));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
       // eventMap.put("High", placeConeHigh(container.getArm(), container.getIntake()));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        //Command leg2 = RobotContainer.getSwerveAutoBuilder().fullAuto(trajectory2HAB);
        return placeConeHigh(container.getArm(), container.getIntake())
            .andThen(fullAuto)
            .andThen(new AutoBalanceCommand(container.getDrivetrain()));

        
    }

    private static Command createMiddleHighEngage(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("MiddleHighEngage", new PathConstraints(0.8 * MAX_AUTO_VELOCITY_METERS_PER_SECOND, 0.8*MAX_AUTO_ACCELERATION_METERS_PER_SECOND));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
       // eventMap.put("High", placeConeHigh(container.getArm(), container.getIntake()));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        //Command leg2 = RobotContainer.getSwerveAutoBuilder().fullAuto(trajectory2HAB);
        return placeCubeHigh(container.getArm(), container.getIntake())
            .andThen(fullAuto)
            .andThen(new AutoBalanceCommand(container.getDrivetrain()));

        
    }

    private static Command createBumperHighEngage(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("BumperHighEngage", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_ACCELERATION_METERS_PER_SECOND));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
       // eventMap.put("High", placeConeHigh(container.getArm(), container.getIntake()));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        //Command leg2 = RobotContainer.getSwerveAutoBuilder().fullAuto(trajectory2HAB);
        return placeConeHigh(container.getArm(), container.getIntake())
            .andThen(fullAuto)
            .andThen(new AutoBalanceCommand(container.getDrivetrain()));

        
    }

    private static Command createMidPlace(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Mid Place", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_ACCELERATION_METERS_PER_SECOND));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
        SequentialCommandGroup mid = new SequentialCommandGroup(new ArmRotate(container.getArm(), 0),
            new ArmExtend(container.getArm(), ExtensionSetpoints.MID),
            new ArmRotate(container.getArm(), RotationSetpoints.MID_RADIANS));
        SequentialCommandGroup dropCone = new SequentialCommandGroup(new RunCommand(container.getIntake()::pickUpCube).withTimeout(1),
            new ArmRotate(container.getArm(), RotationSetpoints.MID_RADIANS));
        eventMap.put("Mid", mid);
        eventMap.put("Drop Cone", dropCone);
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    private static Command createEngage(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Engage", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_ACCELERATION_METERS_PER_SECOND));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
        //eventMap.put("Balance", new AutoBalanceCommand(container.getDrivetrain()));
        //eventMap.put("X", new RunCommand(container.getDrivetrain()::setX));
        //eventMap.put("intakeDown", new IntakeDown());

        Command path = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        Command fullAuto = path.andThen( new AutoBalanceCommand(container.getDrivetrain()));            
         
        return fullAuto;
    }

    public static Command create2HAB(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2HAB", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_ACCELERATION_METERS_PER_SECOND));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    private static Command createStraight(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Straight", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_ACCELERATION_METERS_PER_SECOND));
        HashMap<String, Command> eventMap = RobotContainer.getEventMap();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        Command fullAuto = RobotContainer.getSwerveAutoBuilder().fullAuto(pathGroup);
        return fullAuto;
    }

    private static Command createMiddleMidEngage(RobotContainer container) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("MiddleMidEngage", new PathConstraints(MAX_AUTO_VELOCITY_METERS_PER_SECOND, MAX_AUTO_ACCELERATION_METERS_PER_SECOND));
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

    public static Command placeConeHigh(Arm arm, Intake intake){
       return new ArmExtendAndRotate(arm, ArmConstants.ExtensionSetpoints.HIGH, ArmConstants.RotationSetpoints.HIGH_RADIANS)
            .andThen(new InstantCommand(intake::pickUpCube)
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(intake::stop))
            .andThen(new ArmExtendAndRotate(arm,0, Math.PI/2).withTimeout(1.3))
        );
    }

    public static Command placeCubeHigh(Arm arm, Intake intake){
        return new ArmExtendAndRotate(arm, ArmConstants.ExtensionSetpoints.HIGH, ArmConstants.RotationSetpoints.HIGH_RADIANS)
             .andThen(new InstantCommand(intake::pickUpCone)
             .andThen(new WaitCommand(0.5))
             .andThen(new InstantCommand(intake::stop))
             .andThen(new ArmExtendAndRotate(arm,0, Math.PI/2).withTimeout(1.3))
         );
     }


}

