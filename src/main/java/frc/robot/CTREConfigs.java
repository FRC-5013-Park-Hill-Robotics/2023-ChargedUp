package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.constants.DrivetrainConstants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            DrivetrainConstants.angleEnableCurrentLimit, 
            DrivetrainConstants.angleContinuousCurrentLimit, 
            DrivetrainConstants.anglePeakCurrentLimit, 
            DrivetrainConstants.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = DrivetrainConstants.angleKP;
        swerveAngleFXConfig.slot0.kI = DrivetrainConstants.angleKI;
        swerveAngleFXConfig.slot0.kD = DrivetrainConstants.angleKD;
        swerveAngleFXConfig.slot0.kF = DrivetrainConstants.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            DrivetrainConstants.driveEnableCurrentLimit, 
            DrivetrainConstants.driveContinuousCurrentLimit, 
            DrivetrainConstants.drivePeakCurrentLimit, 
            DrivetrainConstants.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = DrivetrainConstants.driveKP;
        swerveDriveFXConfig.slot0.kI = DrivetrainConstants.driveKI;
        swerveDriveFXConfig.slot0.kD = DrivetrainConstants.driveKD;
        swerveDriveFXConfig.slot0.kF = DrivetrainConstants.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = DrivetrainConstants.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = DrivetrainConstants.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = DrivetrainConstants.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}