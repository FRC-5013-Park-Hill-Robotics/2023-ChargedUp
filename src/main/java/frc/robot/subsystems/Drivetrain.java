// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.DrivetrainConstants.PIGEON_ID;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import static frc.robot.constants.DrivetrainConstants.MAX_VOLTAGE;



import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import frc.robot.SwerveModule;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.FieldTrajectoryConstants;
import frc.robot.constants.DrivetrainConstants.TranslationGains;


public class Drivetrain extends SubsystemBase {
	private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(PIGEON_ID.id,PIGEON_ID.busName);
	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			// Front left
			new Translation2d(DrivetrainConstants.trackWidth / 2.0, DrivetrainConstants.wheelBase/ 2.0),
			// Front right
			new Translation2d(DrivetrainConstants.trackWidth / 2.0, -DrivetrainConstants.wheelBase / 2.0),
			// Back left
			new Translation2d(-DrivetrainConstants.trackWidth / 2.0, DrivetrainConstants.wheelBase / 2.0),
			// Back right
			new Translation2d(-DrivetrainConstants.trackWidth / 2.0, -DrivetrainConstants.wheelBase / 2.0));

	// FIX We need to figure out initial possition.
	private Pose2d m_pose = new Pose2d();
	private SwerveDrivePoseEstimator m_PoseEstimator ;
	private PIDController balancePID = new PIDController(0.012, 0, .008);
	private double oldPitch;
	// These are our modules. We initialize them in the constructor.
	public SwerveModule[] mSwerveMods;

	private SwerveModuleState[] m_desiredStates;

	private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(TranslationGains.kS, TranslationGains.kV,
			TranslationGains.kA);

	public Drivetrain() {
			
		zeroGyroscope();
		mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, DrivetrainConstants.Mod0.constants),
            new SwerveModule(1, DrivetrainConstants.Mod1.constants),
            new SwerveModule(2, DrivetrainConstants.Mod2.constants),
            new SwerveModule(3, DrivetrainConstants.Mod3.constants)
        };
		Timer.delay(1.0);
        resetModulesToAbsolute();

		m_PoseEstimator  = new SwerveDrivePoseEstimator(m_kinematics,
		getYawR2d(),
		getModulePositions(),
		new Pose2d(),
		VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
		VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

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

	public void resetPose(Pose2d newPosition) {
		setInitialPosition(newPosition, newPosition.getRotation());
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
		setDesiredStates(m_desiredStates,true);
	}
	public void driveClosedLoop(ChassisSpeeds chassisSpeeds) {
		if (m_desiredStates != null && chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0
				&& chassisSpeeds.omegaRadiansPerSecond == 0) {
			m_desiredStates[0].speedMetersPerSecond = 0;
			m_desiredStates[1].speedMetersPerSecond = 0;
			m_desiredStates[2].speedMetersPerSecond = 0;
			m_desiredStates[3].speedMetersPerSecond = 0;
		} else {
			m_desiredStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
		}
		setDesiredStates(m_desiredStates,false);
	}

	public void setDesiredStates(SwerveModuleState[] newStates, boolean isOpenLoop) {
		for(SwerveModule mod : mSwerveMods){
	          mod.setDesiredState(newStates[mod.moduleNumber], isOpenLoop);
        }
	}

	public void setDesiredStates(SwerveModuleState[] newStates) {
		setDesiredStates(newStates,false);
	}

	@Override
    public void periodic(){
        updatePoseEstimator(); 
		SmartDashboard.putNumber("Gyro", getYawR2d().getDegrees());
		SmartDashboard.putNumber("pitch", getPitchR2d().getDegrees());
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

	private void updatePoseEstimator() {
		m_pose = m_PoseEstimator.update(Rotation2d.fromDegrees(m_pigeon.getYaw()),getModulePositions());
		SmartDashboard.putNumber("Pose x", m_pose.getX());
		SmartDashboard.putNumber("Pose y", m_pose.getY());
	}

	public void updatePoseEstimator(PhotonPoseEstimator photonEstimate){
		Optional<EstimatedRobotPose>  pose = photonEstimate.update();
		if (!pose.isEmpty()){
			m_PoseEstimator.addVisionMeasurement(pose.get().estimatedPose.toPose2d(),pose.get().timestampSeconds);
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
		return DrivetrainConstants.maxSpeed  * percentOutput;
	}

	public static double percentOutputToRadiansPerSecond(double percentOutput) {
		return DrivetrainConstants.maxAngularVelocity * percentOutput;
	}

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

	public void setX() {
		m_desiredStates[0].speedMetersPerSecond = 0;
		m_desiredStates[1].speedMetersPerSecond = 0;
		m_desiredStates[2].speedMetersPerSecond = 0;
		m_desiredStates[3].speedMetersPerSecond = 0;

		m_desiredStates[0].angle = Rotation2d.fromDegrees(45);
		m_desiredStates[1].angle = Rotation2d.fromDegrees(135);
		m_desiredStates[2].angle = Rotation2d.fromDegrees(225);
		m_desiredStates[3].angle = Rotation2d.fromDegrees(315);

		setDesiredStates(m_desiredStates,true);

	}
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
	public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

	public void engage(Pose2d pose, Alliance alliance) {
		if ((pose.getY() > FieldTrajectoryConstants.blueChargingStationMiddleYMeters -0.5) && (pose.getY() < FieldTrajectoryConstants.blueChargingStationMiddleYMeters+0.5) && (pose.getX() > FieldTrajectoryConstants.chargingStationMiddleXMeters -0.5) && (pose.getX() <  FieldTrajectoryConstants.chargingStationMiddleXMeters +0.5) && (Alliance.Blue == alliance) ) {
			setX();
		}
		else if ((pose.getY() > FieldTrajectoryConstants.redChargingStationMiddleYMeters -0.5) && (pose.getY() < FieldTrajectoryConstants.redChargingStationMiddleYMeters+0.5) && (pose.getX() > FieldTrajectoryConstants.chargingStationMiddleXMeters -0.5) && (pose.getX() <  FieldTrajectoryConstants.chargingStationMiddleXMeters +0.5) && (Alliance.Red == alliance) ) {
			setX();
		}
	}

	public void balance() {
		double pitch = getPitchR2d().getDegrees();
		if (Math.abs(pitch) < Math.abs(oldPitch)){
			//it is getting better so wait.
			drive(0,0,0);
		} else {
			//drive 
			double xPower = MathUtil.clamp(balancePID.calculate(pitch), -0.15, 0.15);
			drive(-xPower, 0, 0);
		}
		oldPitch = pitch;
	}

	public void resetBalance(){
		balancePID.setSetpoint(0);
		balancePID.setTolerance(2);
		balancePID.reset();
	}
	public void drive(double x, double y, double rotation) {
		ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				x * DrivetrainConstants.maxSpeed,
				y * DrivetrainConstants.maxSpeed,
				rotation * DrivetrainConstants.maxAngularVelocity,
				getYawR2d());

		drive(chassisSpeeds);
	}
}