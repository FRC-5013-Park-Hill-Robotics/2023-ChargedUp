package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DrivetrainConstants;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.DrivetrainConstants.ThetaGains;
import frc.robot.constants.DrivetrainConstants.TranslationGains;
import frc.robot.subsystems.Drivetrain;
import frc.robot.trobot5013lib.command.TrajectoryLogging;

public class AutonomousCommandFactory {
	public static double MAX_AUTO_VELOCITY_METERS_PER_SECOND = DrivetrainConstants.maxSpeed/2;
	public static double MAX_AUTO_ANGULAR_RADIANS_PER_SECOND = DrivetrainConstants.maxAngularVelocity/2;
	public static String path2HAB = "2HAB";
	public static String pathCurvy = "Curvy";

	public static final String[] AUTOS = {path2HAB, pathCurvy};

	public static Command createStartupCommand(RobotContainer container, PathPlannerTrajectory trajectory) {
		Drivetrain drivetrain = container.getDrivetrain();
		return new ParallelCommandGroup(
				new InstantCommand(() -> drivetrain.setInitialPosition(trajectory.getInitialPose(),
					trajectory.getInitialState().holonomicRotation))
				//new InstantCommand(container.getintake()::dropIntake)
			);
	}
	
	public static PPSwerveControllerCommand createSwerveControllerCommand(PathPlannerTrajectory trajectory,
			Drivetrain drivetrain) {
		PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(trajectory,
				drivetrain::getPose,
				drivetrain.getKinematics(),
				new PIDController(TranslationGains.kP, TranslationGains.kI, TranslationGains.kD),
				new PIDController(TranslationGains.kP, TranslationGains.kI, TranslationGains.kD),
				new PIDController(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD),
				drivetrain::setDesiredStates, drivetrain);
		return swerveControllerCommand;
	}

	public static Command createAutonomous(RobotContainer container, String name) {
		if (path2HAB.equals(name)) {
			return create2HAB(container);
		}
		else {
			return createCurvy(container);
		}
	}
	

	public static Command create2HAB(RobotContainer container) {
		Drivetrain drivetrain = container.getDrivetrain();

		PathPlannerTrajectory leg1Trajectory = PathPlanner.loadPath("2HAB",
				MAX_AUTO_VELOCITY_METERS_PER_SECOND,
				MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);

		Command leg1 = createSwerveControllerCommand(leg1Trajectory, drivetrain).raceWith(new TrajectoryLogging(leg1Trajectory, drivetrain::getPose));
		Command startup = createStartupCommand(container, leg1Trajectory);
		

		return new SequentialCommandGroup(
				startup,
				leg1);
	}


	public static Command createCurvy(RobotContainer container) {
		Drivetrain drivetrain = container.getDrivetrain();

		PathPlannerTrajectory leg1Trajectory = PathPlanner.loadPath("Curvy",
				MAX_AUTO_VELOCITY_METERS_PER_SECOND,
				MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);

		Command leg1 = createSwerveControllerCommand(leg1Trajectory, drivetrain).raceWith(new TrajectoryLogging(leg1Trajectory, drivetrain::getPose));
		Command startup = createStartupCommand(container, leg1Trajectory);
		

		return new SequentialCommandGroup(
				startup,
				leg1);
	}
}
