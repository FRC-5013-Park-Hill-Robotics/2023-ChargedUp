// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.CANConstants;
import frc.robot.constants.ArmConstants.*;
import frc.robot.trobot5013lib.RevThroughBoreEncoder;

import static frc.robot.constants.ArmConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.util.Units;



/** Add your docs here. */
public class Arm extends SubsystemBase {
    private final TalonFX m_extensionMotor = new TalonFX(CANConstants.EXTENSION_ID);
    private final TalonFX m_rotationMotor = new TalonFX(CANConstants.SHOULDER_ID);
    private final PIDController m_extensionPIDController = new PIDController(ExtensionGains.kP, ExtensionGains.kI, ExtensionGains.kD);
    private final Constraints m_rotationConstraints = new Constraints(RotationConstraints.MAX_ROTATION_VELOCITY_RPS, RotationConstraints.MAX_ROTATION_ACCELERATION_RPSPS);
    private final PIDController m_rotationPIDController = new PIDController(RotationGains.kP, RotationGains.kI, RotationGains.kD);
    private final AnalogPotentiometer m_potentiometer = new AnalogPotentiometer(0);
    private final RevThroughBoreEncoder m_angleEncoder = new RevThroughBoreEncoder(CANConstants.ARM_ANGLE_ENCODER);
    private ArmFeedforward m_rotationFeedForward = new ArmFeedforward(RotationGains.kS, RotationGains.kG, RotationGains.kV); 
    private SimpleMotorFeedforward m_extensionFeedForward = new SimpleMotorFeedforward(ExtensionGains.kS, ExtensionGains.kA, ExtensionGains.kA);
    private double angleSetpointRadians ;
    private boolean isOpenLoopRotation = true;
    private boolean isOpenLooExtension = true;
    private double extensionSetpoint; 


    public Arm() {
        m_angleEncoder.setInverted(true);
        m_angleEncoder.setOffset(ArmConstants.ARM_OFFSET_DEGREES);
        setAngleSetpointRadians(getArmAngleRadians());
       
        m_extensionMotor.configFactoryDefault();
        m_extensionMotor.setNeutralMode(NeutralMode.Brake);
        m_extensionMotor.setInverted(true);
        resetExtensionEncoder();
        setExtensionSetpoint(getCurrentExtensionDistance());
        m_extensionMotor.setSensorPhase(true);
        m_extensionPIDController.setTolerance(0.03);
        m_extensionPIDController.disableContinuousInput();
        isOpenLooExtension = false;

        new Trigger(this::isExtensionCurrentSpike).onTrue(new InstantCommand(this::resetExtensionEncoder));
       
        m_rotationPIDController.enableContinuousInput(0, 2 * Math.PI);
        m_rotationPIDController.setTolerance(RotationGains.TOLERANCE.getRadians());

        m_rotationMotor.configFactoryDefault();
        m_rotationMotor.setNeutralMode(NeutralMode.Brake);
        m_rotationMotor.setInverted(false);
        isOpenLoopRotation = false;

  
        SmartDashboard.putData("Arm Rotation PID Controller", m_rotationPIDController);
    }

    
    public double  getExtensionSetpoint() {
        return extensionSetpoint;
    }

    public void setExtensionSetpoint(double extensionSetpoint) {
 
        this.extensionSetpoint = extensionSetpoint;
    
    }
	public PIDController getExtensionPIDController() {
		return m_extensionPIDController;
	}

    public PIDController getRotationPIDController() {
        return m_rotationPIDController;
    }
    public Command rotateToCommand(Rotation2d angle) {
        return runOnce(() -> setAngleSetpointRadians(angle.getRadians()));
    }

    public void rotate(double percent) {
        SmartDashboard.putNumber("RotateRotercent", percent);
        if (percent == 0.0){
            if (isOpenLoopRotation){
                hold();
            }
        } else {
            isOpenLoopRotation = true;
            m_rotationMotor.set(ControlMode.PercentOutput, percent);
        }
    }

    public Command extendToCommand(double length) {
        m_extensionPIDController.setTolerance(length);
        return runOnce(() -> setExtensionSetpoint(length));
    }

    public Command extendAndRotateCommand(Rotation2d angle, double length) {
        m_extensionPIDController.setTolerance(length);
        return run(() -> setExtensionAndRotation(angle.getRadians(),length))
        .until(this::isExtenstionAndRotationAtSetpoint)
        .andThen(runOnce(() -> holdExtension()));
    }

    public void setExtensionAndRotation(double angle, double length){
       setExtensionSetpoint(length);
       setAngleSetpointRadians(angle);
    }

    public boolean isExtenstionAndRotationAtSetpoint(){
        return m_extensionPIDController.atSetpoint() && m_rotationPIDController.atSetpoint();
    }

    public void extend(double percent) {
        SmartDashboard.putNumber("Extendercent", percent);
        if (percent == 0.0){
            if (isOpenLooExtension){
                holdExtension();
            }
        } else {
            isOpenLooExtension = true;
            m_extensionMotor.set(ControlMode.PercentOutput, percent);
            
        }
       
    }

    public void extendClosedLoop(double velocity) {
        isOpenLooExtension = false;
        double feedForward = m_extensionFeedForward.calculate(velocity);
        if (velocity < 0){
            feedForward = feedForward - 1.56;
        }
        SmartDashboard.putNumber("ExVel", velocity);
        SmartDashboard.putNumber("ExFeedForward", feedForward);
        SmartDashboard.putNumber("ExVoltage",RobotContainer.voltageToPercentOutput(feedForward));
        
        m_extensionMotor.set(ControlMode.PercentOutput, RobotContainer.voltageToPercentOutput(feedForward));
    }

    public void rotateClosedLoop(double velocity) {
        isOpenLoopRotation = false;
        SmartDashboard.putNumber("OUTPUT", velocity);
        double feedForward = m_rotationFeedForward.calculate(getArmAngleRadians(),velocity);
        SmartDashboard.putNumber("FeedForward", feedForward);
        SmartDashboard.putNumber("Voltage",RobotContainer.voltageToPercentOutput(feedForward));
        m_rotationMotor.set(ControlMode.PercentOutput, RobotContainer.voltageToPercentOutput(feedForward));
    }

    public double getArmAngleRadians(){
        return m_angleEncoder.getAngle().getRadians();
    }

    public double getAngleSetpointRadians() {
        return angleSetpointRadians;
    }

    public void setAngleSetpointRadians(double angleSetpoint) {
        this.angleSetpointRadians = angleSetpoint;
    }

    public void hold(){
        setAngleSetpointRadians(getArmAngleRadians());
        isOpenLoopRotation = false;
    }

    public void holdExtension(){
        setExtensionSetpoint(getCurrentExtensionDistance());
        isOpenLooExtension = false;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Angle", (m_angleEncoder.getAngle()).getDegrees());
        SmartDashboard.putNumber("Setpoint", Units.radiansToDegrees(getAngleSetpointRadians()));
        SmartDashboard.putNumber("Measurement", Units.radiansToDegrees(getArmAngleRadians()));
        SmartDashboard.putNumber("Error", Units.radiansToDegrees(getAngleSetpointRadians() - getArmAngleRadians()) );
        SmartDashboard.putNumber("Extension Sensor  Position",m_extensionMotor.getSelectedSensorPosition());
        SmartDashboard.putBoolean("isOpenLoopRotation", isOpenLoopRotation);
        SmartDashboard.putNumber("Extension Meters", getCurrentExtensionDistance());
        SmartDashboard.putNumber("Extension Setpoint", getExtensionSetpoint());
        SmartDashboard.putNumber("Extension Error", getExtensionSetpoint()-getCurrentExtensionDistance());
        if (isOpenLoopRotation) {
            m_rotationPIDController.reset();
        } else {

            m_rotationPIDController.setSetpoint(getAngleSetpointRadians());
            rotateClosedLoop(m_rotationPIDController.calculate(getArmAngleRadians()));
        }

        if (isOpenLooExtension) {
            m_extensionPIDController.reset();
        } else {
            m_extensionPIDController.setSetpoint(getExtensionSetpoint());
            extendClosedLoop(m_extensionPIDController.calculate(getCurrentExtensionDistance()));
        }

    }

    public boolean isExtensionCurrentSpike(){
        return m_extensionMotor.getStatorCurrent() > 30;
    }

    public void resetExtensionEncoder(){
        m_extensionMotor.setSelectedSensorPosition(0);
    }

    
    public double getCurrentExtensionDistance(){
        return m_extensionMotor.getSelectedSensorPosition()/PULSES_PER_METER_EXTENSION;
    }
}

