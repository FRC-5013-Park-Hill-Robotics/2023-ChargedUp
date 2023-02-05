// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.GlobalConstants.*;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.GamepadDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonVision;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants.TranslationGains;
import frc.robot.constants.DrivetrainConstants.ThetaGains;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static RobotContainer instance;
	private final Map<String, Command> eventMap = new HashMap<>();
	private final Drivetrain m_drivetrainSubsystem = new Drivetrain();
	private final SwerveAutoBuilder m_autoBuilder = new SwerveAutoBuilder(
            m_drivetrainSubsystem::getPose, // Pose2d supplier
            m_drivetrainSubsystem::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
            new PIDConstants(TranslationGains.kP, TranslationGains.kI, TranslationGains.kD), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD), // PID constants to correct for rotation error (used to create the rotation controller)
            m_drivetrainSubsystem::driveClosedLoop, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            m_drivetrainSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        ); 
	private final LogitechController m_controller = new LogitechController(ControllerConstants.DRIVER_CONTROLLER_PORT);
	private final LogitechController m_operator_controller = new LogitechController(
			ControllerConstants.OPERATOR_CONTROLLER_PORT);

	private PowerDistribution m_PowerDistribution = new PowerDistribution(PCM_ID, ModuleType.kRev);
	private PneumaticsControlModule m_pneumaticsHub = new PneumaticsControlModule(PNEUMATICS_HUB);
	private PhotonVision m_photonVision;// = new PhotonVision();
	public static RobotContainer getInstance(){
		return instance;
	}


	/**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
		instance = this;
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        m_drivetrainSubsystem.setDefaultCommand(new GamepadDrive(m_drivetrainSubsystem, m_controller));
		// Configure the button bindings
        configureButtonBindings();
		m_pneumaticsHub.enableCompressorDigital();
	
		SmartDashboard.putStringArray("Auto List",PathPlannerCommandFactory.AUTOS );

		LiveWindow.disableAllTelemetry();
    }

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		// Back button zeros the gyroscope

		
		//new Button(m_operator_controller::getXButton).whenHeld(new AutoClimber(m_Climber, m_drivetrainSubsystem.getPitchR2d()::getDegrees));
		//new Button(m_operator_controller::getBButton).whenPressed(new PreClimbCommand(this));
		// programmer controls
		/*	new Button(m_programmer_controller::getBButton).whileHeld(new InstantCommand(m_shooter::fire))
				.whenReleased(new InstantCommand(m_shooter::stopFiring));
		new Button(m_programmer_controller::getYButton).whileHeld(new InstantCommand(m_conveyor::start));
		new Button(m_programmer_controller::getXButton).whileHeld(new InstantCommand(m_intake::start));// .whenReleased(new
																									// InstantCommand(m_intake::stop));
		new Button(m_programmer_controller::getDPadRight)
				.whenPressed(new InstantCommand(() -> m_shooter.changeSpeed(100)));
		new Button(m_programmer_controller::getDPadLeft)
				.whenPressed(new InstantCommand(() -> m_shooter.changeSpeed(-100)));
		new Button(m_programmer_controller::getLeftBumper).whileHeld(new InstantCommand(m_intake::dropIntake))
				.whenReleased(new InstantCommand(m_intake::raiseIntake));*/
				
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		String autoName = SmartDashboard.getString("Auto Selector", PathPlannerCommandFactory.path2HAB);
		return PathPlannerCommandFactory.createAutonomous(this,autoName);
	}

	public boolean isRedAlliance() {
		return DriverStation.getAlliance() == Alliance.Red;
	}

	public Alliance getAlliance(){
		return DriverStation.getAlliance();
	}

	public boolean isDisabled() {
		return DriverStation.isDisabled();
	}

	public boolean isAutonomous() {
		return DriverStation.isAutonomous();
	}

	public boolean isTeleop() {
		return DriverStation.isTeleop();
	}

	public Drivetrain getDrivetrain() {
		return m_drivetrainSubsystem;
	}

	public static SwerveAutoBuilder getSwerveAutoBuilder() {
		return getInstance().m_autoBuilder;
	}

	public static Map<String, Command> getEventMap() {
		return getInstance().eventMap;
	}

	public LogitechController getcontroller() {
		return m_controller;
	}

	public PowerDistribution getPowerDistribution() {
		return m_PowerDistribution;
	}

	public void setPowerDistribution(PowerDistribution m_PowerDistribution) {
		this.m_PowerDistribution = m_PowerDistribution;
	}

	public PhotonVision getPhotonVision(){
		return m_photonVision;
	}
	public PneumaticsControlModule getPneumaticsHub() {
		return this.m_pneumaticsHub;
	}
}

