// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GlobalConstants.IntakeConstants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class Intake extends SubsystemBase {
    private final CANSparkMax m_flexMotor = new CANSparkMax(0, MotorType.kBrushless);
    private final CANSparkMax m_intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
    private final PIDController m_flexPIDController = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
    private final DutyCycleEncoder m_angleEncoder = new DutyCycleEncoder(0);
    private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV); 


    public Intake() {
        setDefaultCommand(balanceAngle(IntakeConstants.INTAKE_ANGLE_DEGREES));


    } 

    public void pickUpCone() {

    }

    public void pickUpCube() {

    }

    public void flexClosedLoop(double velocity) {
        double feedForward = m_feedForward.calculate(velocity, velocity, velocity); //calculate feed forward
        double percentOutput = 0;//calculate percent out from feed forward
        m_flexMotor.set(percentOutput);
        m_flexMotor.set(RobotContainer.voltageToPercentOutput(feedForward));
    }

    public Command balanceAngle(Rotation2d angle) {
        m_flexPIDController.setTolerance(angle.getRadians());
        return run(() -> flexClosedLoop(m_flexPIDController.calculate(m_angleEncoder.get(), angle.getRadians())))
        .until(m_flexPIDController::atSetpoint)
        .andThen(runOnce(() -> flexClosedLoop(0)));

    }

    
    }


