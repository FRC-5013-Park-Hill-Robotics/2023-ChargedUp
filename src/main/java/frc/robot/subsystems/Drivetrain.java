// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.DrivetrainConstants.PIGEON_ID;
import static frc.robot.constants.DrivetrainConstants.SWERVE_GEAR_RATIO;
import static frc.robot.constants.DrivetrainConstants.MAX_VOLTAGE;



import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import frc.robot.constants.DrivetrainConstants.BackLeftSwerveConstants;
import frc.robot.constants.DrivetrainConstants.BackRightSwerveConstants;
import frc.robot.constants.DrivetrainConstants.FrontLeftSwerveConstants;
import frc.robot.constants.DrivetrainConstants.FrontRightSwerveConstants;
import frc.robot.constants.DrivetrainConstants.TranslationGains;
import frc.robot.constants.DrivetrainConstants.DrivetrainGeometry;

public class Drivetrain extends SubsystemBase {
	private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(PIGEON_ID.id,PIGEON_ID.busName);
	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			// Front left
			new Translation2d(DrivetrainGeometry.TRACKWIDTH_METERS / 2.0, DrivetrainGeometry.WHEELBASE_METERS / 2.0),
			// Front right
			new Translation2d(DrivetrainGeometry.TRACKWIDTH_METERS / 2.0, -DrivetrainGeometry.WHEELBASE_METERS / 2.0),
			// Back left
			new Translation2d(-DrivetrainGeometry.TRACKWIDTH_METERS / 2.0, DrivetrainGeometry.WHEELBASE_METERS / 2.0),
			// Back right
			new Translation2d(-DrivetrainGeometry.TRACKWIDTH_METERS / 2.0, -DrivetrainGeometry.WHEELBASE_METERS / 2.0));

	// FIX We need to figure out initial possition.
	private Pose2d m_pose = new Pose2d();
	private SwerveDrivePoseEstimator m_PoseEstimator ;

	// These are our modules. We initialize them in the constructor.
	private final SwerveModule m_frontLeftModule;
	private final SwerveModule m_frontRightModule;
	private final SwerveModule m_backLeftModule;
	private final SwerveModule m_backRightModule;

	private SwerveModuleState[] m_desiredStates;

	private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(TranslationGains.kS, TranslationGains.kV,
			TranslationGains.kA);

	public Drivetrain() {
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

		Mk4ModuleConfiguration config = new Mk4ModuleConfiguration();
		config.setDriveCurrentLimit(60);
		config.setSteerCurrentLimit(40);
		// Creating the SwerveModules using SDS factory method.
		m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
				tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
				config,	SWERVE_GEAR_RATIO, FrontLeftSwerveConstants.DRIVE_MOTOR_ID, 
				FrontLeftSwerveConstants.STEER_MOTOR_ID,
				FrontLeftSwerveConstants.ENCODER_ID, FrontLeftSwerveConstants.ENCODER_OFFSET_RADIANS);

		m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
				tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
				config,SWERVE_GEAR_RATIO, FrontRightSwerveConstants.DRIVE_MOTOR_ID,
				FrontRightSwerveConstants.STEER_MOTOR_ID,
				FrontRightSwerveConstants.ENCODER_ID, FrontRightSwerveConstants.ENCODER_OFFSET_RADIANS);

		m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
				config,SWERVE_GEAR_RATIO, BackLeftSwerveConstants.DRIVE_MOTOR_ID,
				BackLeftSwerveConstants.STEER_MOTOR_ID,
				BackLeftSwerveConstants.ENCODER_ID, BackLeftSwerveConstants.ENCODER_OFFSET_RADIANS);

		m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
				config,SWERVE_GEAR_RATIO, BackRightSwerveConstants.DRIVE_MOTOR_ID,
				BackRightSwerveConstants.STEER_MOTOR_ID,
				BackRightSwerveConstants.ENCODER_ID, BackRightSwerveConstants.ENCODER_OFFSET_RADIANS);

		m_PoseEstimator  = new SwerveDrivePoseEstimator(m_kinematics,
		getYawR2d(),
		new SwerveModulePosition[] {
			m_frontLeftModule.getPosition(),
			m_frontRightModule.getPosition(),
			m_backLeftModule.getPosition(),
			m_backRightModule.getPosition()
		},
		new Pose2d(),
		VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
		VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
		zeroGyroscope();
	}

	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction the
	 * robot is currently facing to the 'forwards' direction.
	 */
	public void zeroGyroscope() {
		m_pigeon.setYaw(0.0);

	}

	public void setInitialPosition(Pose2d newPosition, Rotation2d newRotation) {
		m_pigeon.setYaw(newRotation.getDegrees());
		m_pose = new Pose2d(
			newPosition.getTranslation(),
			newRotation  );
		m_PoseEstimator = new SwerveDrivePoseEstimator(m_kinematics, getYawR2d(), getModulePositions(), m_pose);
	}

	/*
	 * Return the gyroscope's heading as a Rotation2d object
	 */
	public Rotation2d getYawR2d() {
		return getPose().getRotation();
		//eturn Rotation2d.fromDegrees(m_pigeon.getYaw());
	}

    public Rotation2d getRollR2d() {
		return Rotation2d.fromDegrees(m_pigeon.getRoll());
	}
    public Rotation2d getPitchR2d() {
        return Rotation2d.fromDegrees(m_pigeon.getPitch());
    }

	/*
	 * Return the gyroscope's heading in Radians
	 */
	public double getHeadingRadians() {
		return getYawR2d().getRadians();
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		if (m_desiredStates != null && chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0
				&& chassisSpeeds.omegaRadiansPerSecond == 0) {
			m_desiredStates[0].speedMetersPerSecond = 0;
			m_desiredStates[1].speedMetersPerSecond = 0;
			m_desiredStates[2].speedMetersPerSecond = 0;
			m_desiredStates[3].speedMetersPerSecond = 0;
		} else {
			m_desiredStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
		}
		setDesiredStates(m_desiredStates);
	}

	public void setDesiredStates(SwerveModuleState[] newStates) {
		m_desiredStates = newStates;
		updatePoseEstimator();
		updateDriveStates(m_desiredStates);
	}

	@Override
	public void periodic() {
		updatePoseEstimator();
		SmartDashboard.putNumber("Gyro", getYawR2d().getDegrees());
		SmartDashboard.putNumber("pitch", getPitchR2d().getDegrees());
	}

	private void updatePoseEstimator() {
		m_pose = m_PoseEstimator.update(Rotation2d.fromDegrees(m_pigeon.getYaw()),getModulePositions());
		SmartDashboard.putNumber("Pose x", m_pose.getX());
		SmartDashboard.putNumber("Pose y", m_pose.getY());
	}

	private void updateDriveStates(SwerveModuleState[] desiredStates) {
		if (desiredStates != null) {
			SwerveModuleState frontLeftState = desiredStates[FrontLeftSwerveConstants.STATES_INDEX];
			SwerveModuleState frontRightState = desiredStates[FrontRightSwerveConstants.STATES_INDEX];
			SwerveModuleState backLeftState = desiredStates[BackLeftSwerveConstants.STATES_INDEX];
			SwerveModuleState backRightState = desiredStates[BackRightSwerveConstants.STATES_INDEX];

			SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
					DrivetrainGeometry.MAX_VELOCITY_METERS_PER_SECOND);

			m_frontLeftModule.set(velocityToDriveVolts(frontLeftState.speedMetersPerSecond),
					frontLeftState.angle.getRadians());
			m_frontRightModule.set(velocityToDriveVolts(frontRightState.speedMetersPerSecond),
					frontRightState.angle.getRadians());
			m_backLeftModule.set(velocityToDriveVolts(backLeftState.speedMetersPerSecond),
					backLeftState.angle.getRadians());
			m_backRightModule.set(velocityToDriveVolts(backRightState.speedMetersPerSecond),
					backRightState.angle.getRadians());
		}
	}

	private double velocityToDriveVolts(double speedMetersPerSecond) {
		double ff = m_feedForward.calculate(speedMetersPerSecond);
		return MathUtil.clamp(ff, -MAX_VOLTAGE, MAX_VOLTAGE);
	}

	public Pose2d getPose() {
		return m_pose;
	}

	public SwerveDriveKinematics getKinematics() {
		return m_kinematics;
	}

	public static double percentOutputToMetersPerSecond(double percentOutput) {
		return DrivetrainGeometry.MAX_VELOCITY_METERS_PER_SECOND * percentOutput;
	}

	public static double percentOutputToRadiansPerSecond(double percentOutput) {
		return DrivetrainGeometry.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * percentOutput;
	}

	public SwerveModulePosition[] getModulePositions(){
		return new SwerveModulePosition[]{ m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition() };
	}

	public void setX() {
		m_desiredStates[FrontLeftSwerveConstants.STATES_INDEX].speedMetersPerSecond = 0;
		m_desiredStates[FrontRightSwerveConstants.STATES_INDEX].speedMetersPerSecond = 0;
		m_desiredStates[BackLeftSwerveConstants.STATES_INDEX].speedMetersPerSecond = 0;
		m_desiredStates[BackRightSwerveConstants.STATES_INDEX].speedMetersPerSecond = 0;

		m_desiredStates[FrontLeftSwerveConstants.STATES_INDEX].angle = Rotation2d.fromDegrees(45);
		m_desiredStates[FrontRightSwerveConstants.STATES_INDEX].angle = Rotation2d.fromDegrees(135);
		m_desiredStates[BackLeftSwerveConstants.STATES_INDEX].angle = Rotation2d.fromDegrees(225);
		m_desiredStates[BackRightSwerveConstants.STATES_INDEX].angle = Rotation2d.fromDegrees(315);

		setDesiredStates(m_desiredStates);

	}

}