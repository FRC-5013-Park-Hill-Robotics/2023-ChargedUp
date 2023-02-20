// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.GlobalConstants.IntakeConstants;
import frc.robot.RobotContainer;


/** Add your docs here. */
public class Intake extends SubsystemBase {
    private static final Rotation2d INTAKE_ANGLE_DEGREES = IntakeConstants.INTAKE_ANGLE_DEGREES;
    private final CANSparkMax m_flexMotor = new CANSparkMax(CANConstants.WRIST_ID, MotorType.kBrushless);
    private final CANSparkMax m_intakeMotor = new CANSparkMax(CANConstants.INTAKE_ID, MotorType.kBrushless);
    private final PIDController m_flexPIDController = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
    private final DutyCycleEncoder m_angleEncoder = new DutyCycleEncoder(0);

    private ArmFeedforward m_feedForward = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV); 


    public Intake() {
        


    } 

    public void pickUpCone() {

    }

    public void pickUpCube() {

    }

    public void flexClosedLoop(double velocity, double angleRadians) {
        double feedForward = m_feedForward.calculate(angleRadians,velocity); //calculate feed forward
        double percentOutput = 0;//calculate percent out from feed forward
        m_flexMotor.set(percentOutput);
        m_flexMotor.set(RobotContainer.voltageToPercentOutput(feedForward));
    }

    @Override
    public void periodic(){
        /*  because the flex motor is so fast commented out until we get good PID values and limits 
        so we dont break the wrist
        m_flexPIDController.setTolerance(INTAKE_ANGLE_DEGREES.getRadians());
        if (m_flexPIDController.atSetpoint()){
            flexClosedLoop(0,INTAKE_ANGLE_DEGREES.getRadians());
        } else {
            flexClosedLoop(m_flexPIDController.calculate(m_angleEncoder.get(), INTAKE_ANGLE_DEGREES.getRadians()),INTAKE_ANGLE_DEGREES.getRadians());
        }
        */
    }
    
    }


