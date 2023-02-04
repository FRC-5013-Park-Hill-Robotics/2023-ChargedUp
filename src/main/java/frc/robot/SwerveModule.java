package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.falconSwerveLib.math.Conversions;
import frc.falconSwerveLib.util.CTREModuleState;
import frc.falconSwerveLib.util.SwerveModuleConstants;
import frc.robot.constants.DrivetrainConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrivetrainConstants.driveKS, DrivetrainConstants.driveKV, DrivetrainConstants.driveKA);
  
    public double makePositiveDegrees(double anAngle ){
        double degrees = anAngle;
        degrees = degrees % 360;
        if (degrees < 0.0){
            degrees = degrees + 360;
        }
        return degrees;

    }
    public double makePositiveDegrees(Rotation2d anAngle){
        return makePositiveDegrees(anAngle.getDegrees());
    }
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID.id, moduleConstants.cancoderID.busName);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID.id,moduleConstants.angleMotorID.busName);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID.id,moduleConstants.driveMotorID.busName);
        configDriveMotor();

        lastAngle = Rotation2d.fromDegrees(makePositiveDegrees(getState().angle));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / DrivetrainConstants.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, DrivetrainConstants.wheelCircumference, DrivetrainConstants.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    public Rotation2d optimizeTurn(Rotation2d oldAngle, Rotation2d newAngle){
        double steerAngle = makePositiveDegrees(newAngle);
        steerAngle %= (360);
        if (steerAngle < 0.0) {
            steerAngle += 360;
        }

        double difference = steerAngle - oldAngle.getDegrees();
        // Change the target angle so the difference is in the range [-360, 360) instead of [0, 360)
        if (difference >= 360) {
            steerAngle -= 360;
        } else if (difference < -360) {
            steerAngle += 360;
        }
        difference = steerAngle - oldAngle.getDegrees(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (difference >90 || difference < -90) {
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            steerAngle += 180;
        }

        return Rotation2d.fromDegrees(makePositiveDegrees(steerAngle));
    }
    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DrivetrainConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), DrivetrainConstants.angleGearRatio));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), DrivetrainConstants.angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(makePositiveDegrees(getCanCoder().getDegrees() - angleOffset.getDegrees()), DrivetrainConstants.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(DrivetrainConstants.angleMotorInvert);
        mAngleMotor.setNeutralMode(DrivetrainConstants.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(DrivetrainConstants.driveMotorInvert);
        mDriveMotor.setNeutralMode(DrivetrainConstants.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), DrivetrainConstants.wheelCircumference, DrivetrainConstants.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), DrivetrainConstants.wheelCircumference, DrivetrainConstants.driveGearRatio), 
            getAngle()
        );
    }
}