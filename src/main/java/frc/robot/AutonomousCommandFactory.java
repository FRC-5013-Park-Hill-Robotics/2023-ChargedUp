package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.DrivetrainConstants.DrivetrainGeometry;
import frc.robot.constants.DrivetrainConstants.ThetaGains;
import frc.robot.constants.DrivetrainConstants.TranslationGains;
import frc.robot.subsystems.Drivetrain;
import frc.robot.trobot5013lib.command.TrajectoryLogging;

public class AutonomousCommandFactory {
	public static double MAX_AUTO_VELOCITY_METERS_PER_SECOND = DrivetrainGeometry.MAX_VELOCITY_METERS_PER_SECOND/2;
	public static double MAX_AUTO_ANGULAR_RADIANS_PER_SECOND = DrivetrainGeometry.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND/2;
	public static String BARRIER_STRAIGHT = "Barrier Straight";

	public static final String[] AUTOS = {BARRIER_STRAIGHT};

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

    public static Command createAutonomous(RobotContainer container) {
		return createBarrierStraight(container);
	}

	public static Command createAutonomous(RobotContainer container, String name) {
		if (BARRIER_STRAIGHT.equals(name)) {
			return createBarrierStraight(container);
        }
        return createBarrierStraight(container);

		
		
	}

	public static Command createBarrierStraight(RobotContainer container) {
		Drivetrain drivetrain = container.getDrivetrain();

		PathPlannerTrajectory leg1Trajectory = PathPlanner.loadPath("Barrier Straight",
				MAX_AUTO_VELOCITY_METERS_PER_SECOND,
				MAX_AUTO_VELOCITY_METERS_PER_SECOND / .33);

		Command leg1 = createSwerveControllerCommand(leg1Trajectory, drivetrain).raceWith(new TrajectoryLogging(leg1Trajectory, drivetrain::getPose));
		Command startup = createStartupCommand(container, leg1Trajectory);
		

		return new SequentialCommandGroup(
				startup,
				leg1);
	}
}
