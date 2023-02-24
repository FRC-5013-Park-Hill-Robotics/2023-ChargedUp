// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.CANConstants;
import frc.robot.constants.ArmConstants.*;
import frc.robot.constants.GlobalConstants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.Command;



/** Add your docs here. */
public class Arm extends SubsystemBase {
    private final TalonFX m_extensionMotor = new TalonFX(CANConstants.EXTENSION_ID);
    private final TalonFX m_rotationMotor = new TalonFX(CANConstants.SHOULDER_ID);
    private final PIDController m_extensionPIDController = new PIDController(extensionGains.kP, extensionGains.kI, extensionGains.kD);
    private final PIDController m_rotationPIDController = new PIDController(rotationGains.kP, rotationGains.kI, rotationGains.kD);
    private final AnalogPotentiometer m_potentiometer = new AnalogPotentiometer(0);
    private final DutyCycleEncoder m_angleEncoder = new DutyCycleEncoder(0);
    //private ArmFeedforward m_feedForward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV); 


    public Arm() {
        m_extensionMotor.setNeutralMode(NeutralMode.Brake);
    }

	public PIDController getExtensionPIDController() {
		return m_extensionPIDController;
	}

    public PIDController getRotationPIDController() {
        return m_rotationPIDController;
    }
    

    public void rotate(double percent) {
        m_rotationMotor.set(ControlMode.PercentOutput, percent);
    }

    public Command extendToCommand(double length) {
        m_extensionPIDController.setTolerance(length);
        return run(() -> extendClosedLoop(m_extensionPIDController.calculate(m_potentiometer.get(), length)))
        .until(m_extensionPIDController::atSetpoint)
        .andThen(runOnce(() -> extendClosedLoop(0)));
    }

    public void extend(double percent) {
        m_extensionMotor.set(ControlMode.PercentOutput, percent);
    }

    public void extendClosedLoop(double velocity) {
        double feedForward = 0; //calculate feed forward
        m_extensionMotor.set(ControlMode.PercentOutput, RobotContainer.voltageToPercentOutput(feedForward));
    }

    public void rotateClosedLoop(double velocity) {
        double feedForward = 0; //calculate feed forward
        m_rotationMotor.set(ControlMode.PercentOutput, RobotContainer.voltageToPercentOutput(feedForward));
    }

    public Command rotateToCommand(Rotation2d angle) {
        m_rotationPIDController.setTolerance(angle.getRadians());
        return run(() -> rotateClosedLoop(m_extensionPIDController.calculate(m_angleEncoder.get(), angle.getRadians())))
        .until(m_extensionPIDController::atSetpoint)
        .andThen(runOnce(() -> rotateClosedLoop(0)));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Angle", m_angleEncoder.getAbsolutePosition());
    }

}

