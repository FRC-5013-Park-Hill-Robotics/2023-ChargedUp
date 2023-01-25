// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.GlobalConstants.*;

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
//import frc.robot.subsystems.Intake;

//import frc.robot.subsystems.ShooterVision;
//import frc.robot.subsystems.StatusLED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final Drivetrain m_drivetrainSubsystem = new Drivetrain();
	private final LogitechController m_controller = new LogitechController(ControllerConstants.DRIVER_CONTROLLER_PORT);
	private final LogitechController m_operator_controller = new LogitechController(
			ControllerConstants.OPERATOR_CONTROLLER_PORT);
	//private final LogitechController m_programmer_controller = new LogitechController(
	//		ControllerConstants.PROGRAMMER_CONTROLLER_PORT);

	private PowerDistribution m_PowerDistribution = new PowerDistribution(PCM_ID, ModuleType.kRev);
	private PneumaticsControlModule m_pneumaticsHub = new PneumaticsControlModule(PNEUMATICS_HUB);
	//private StatusLED m_StatusLED = new StatusLED(this);
	//private ShooterVision m_shooterVision = new ShooterVision();
	
	//private Intake m_intake = new Intake(m_conveyor, this);


	/**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        m_drivetrainSubsystem.setDefaultCommand(new GamepadDrive(m_drivetrainSubsystem, m_controller));
		// Configure the button bindings
        configureButtonBindings();
		m_pneumaticsHub.enableCompressorDigital();
	
		SmartDashboard.putStringArray("Auto List",AutonomousCommandFactory.AUTOS );

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
		String autoName = SmartDashboard.getString("Auto Selector", AutonomousCommandFactory.path2HAB);
		return AutonomousCommandFactory.createAutonomous(this,autoName);
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

	public LogitechController getcontroller() {
		return m_controller;
	}

	public PowerDistribution getPowerDistribution() {
		return m_PowerDistribution;
	}

	public void setPowerDistribution(PowerDistribution m_PowerDistribution) {
		this.m_PowerDistribution = m_PowerDistribution;
	}

/*
	public StatusLED getStatusLED() {
		return m_StatusLED;
	}

	public void setStatusLED(StatusLED m_StatusLED) {
		this.m_StatusLED = m_StatusLED;
	}
*/
	//public Intake getintake() {
		//return m_intake;
	//}

	//public void setintake(Intake m_intake) {
		//this.m_intake = m_intake;
	//}

	public PneumaticsControlModule getPneumaticsHub() {
		return this.m_pneumaticsHub;
	}
}

