// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.GlobalConstants.*;

import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.databind.exc.MismatchedInputException;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmBrake;
import frc.robot.commands.ArmControl;
import frc.robot.commands.ArmExtend;
import frc.robot.commands.ArmExtendAndRotate;
import frc.robot.commands.ArmRotate;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.GamepadDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PhotonVision;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DrivetrainConstants.TranslationGains;
import frc.robot.constants.ArmConstants;
import static frc.robot.constants.ArmConstants.*;
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
	private final HashMap<String, Command> eventMap = new HashMap<>();
	private final Drivetrain m_drivetrainSubsystem = new Drivetrain();
	private final Arm m_arm = new Arm();
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
	private Intake m_intake = new Intake(m_arm);
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
		m_arm.setDefaultCommand(new ArmControl(m_arm, m_operator_controller));
		
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
		//double substation 
	/* 	new Trigger(m_operator_controller::getAButton).whileTrue(m_arm.extendAndRotateCommand(
			Rotation2d.fromDegrees(ArmConstants.RotationSetpoints.DOUBLE_SUBSTATION), ArmConstants.ExtensionSetpoints.DOUBLE_SUBSTATION));
	*/
	double x = ExtensionSetpoints.DOUBLE_SUBSTATION;
	
		
		new Trigger(m_controller::getXButton)
			.whileTrue(new RunCommand(m_drivetrainSubsystem::setX,m_drivetrainSubsystem));

		new Trigger(m_controller::getAButton)
			.whileTrue(new AutoBalanceCommand(m_drivetrainSubsystem)
				.andThen(new RunCommand(m_drivetrainSubsystem::setX,m_drivetrainSubsystem)));

		new Trigger(m_controller::getBackButton)
			.onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope));

		new Trigger(m_controller::getStartButton)
			.onTrue(new InstantCommand(m_drivetrainSubsystem::resetModulesToAbsolute));

		//new Trigger(m_controller::getRightBumper)
			//.whileTrue(new AlignToDoubleSubstation())

		//new Trigger(m_operator_controller::getXButton)
			//.whileTrue(new ArmExtend(m_arm, ExtensionSetpoints.DOUBLE_SUBSTATION)
				//.andThen(new ArmRotate(m_arm, RotationSetpoints.DOUBLE_SUBSTATION_RADIANS)));

		//extemd and rotate to double substation
		new Trigger(m_operator_controller::getXButton)
			.whileTrue(new ArmExtendAndRotate(m_arm, ExtensionSetpoints.DOUBLE_SUBSTATION, RotationSetpoints.DOUBLE_SUBSTATION_RADIANS)
			.andThen(new ArmBrake(m_arm)));
		
		//extend and rotate to low node
		new Trigger(m_operator_controller::getAButton)
			.whileTrue(new ArmExtendAndRotate(m_arm, ExtensionSetpoints.LOW, RotationSetpoints.LOW_RADIANS)
			.andThen(new ArmBrake(m_arm)));

		// extend and rotate to mid node
		new Trigger(m_operator_controller::getBButton)
			.whileTrue(new ArmExtendAndRotate(m_arm, ExtensionSetpoints.MID, RotationSetpoints.MID_RADIANS)
			.andThen(new ArmBrake(m_arm)));

		//extend and rotate to high node
		new Trigger(m_operator_controller::getYButton)
			.whileTrue(new ArmExtendAndRotate(m_arm, ExtensionSetpoints.HIGH, RotationSetpoints.HIGH_RADIANS)
			.andThen(new ArmBrake(m_arm)));

		new Trigger(m_operator_controller::getRightBumper)
			.whileTrue(new ArmBrake(m_arm).andThen(new InstantCommand(m_arm::hold)));
		
		new Trigger(m_operator_controller::getLeftBumper)
				.whileTrue(new ArmRotate(m_arm, Math.PI/2).andThen(new ArmBrake(m_arm).andThen(new InstantCommand(m_arm::hold))));

		new Trigger(m_operator_controller::getLeftTriggerButton).whileTrue(new InstantCommand(m_intake::pickUpCube)).onFalse(new InstantCommand(m_intake::stop));
		//spin intake, cube

		new Trigger(m_operator_controller::getRightTriggerButton).whileTrue(new InstantCommand(m_intake::pickUpCone)).onFalse(new InstantCommand(m_intake::stop));
		//spin intake, cone
			
	

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

	public static HashMap<String, Command> getEventMap() {
		return getInstance().eventMap;
	}

	public LogitechController getcontroller() {
		return m_controller;
	}

	public PowerDistribution getPowerDistribution() {
		return m_PowerDistribution;
	}

	public static PowerDistribution getPowerDistributionInstance() {
		return getInstance().m_PowerDistribution;
	}

	public void setPowerDistribution(PowerDistribution m_PowerDistribution) {
		this.m_PowerDistribution = m_PowerDistribution;
	}

	public PhotonVision getPhotonVision(){
		return m_photonVision;
	}
	public static double voltageToPercentOutput(double voltage) {
		return MathUtil.clamp(voltage/Math.min(12, getPowerDistributionInstance().getVoltage()), -1, 1);
	}

	public Arm getArm(){
		return m_arm;
	}

	public Intake getIntake() {
		return m_intake;
	}
/*
	public StatusLED getStatusLED() {
		return m_StatusLED;
	}
	public PneumaticsControlModule getPneumaticsHub() {
		return this.m_pneumaticsHub;
	}
	*/
}

