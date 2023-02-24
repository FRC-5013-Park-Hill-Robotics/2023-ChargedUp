// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.GlobalConstants.IntakeConstants;
import frc.robot.RobotContainer;


/** Add your docs here. */
public class Intake extends SubsystemBase {
    private final CANSparkMax m_flexMotor = new CANSparkMax(CANConstants.WRIST_ID, MotorType.kBrushless);
    private final CANSparkMax m_intakeMotor = new CANSparkMax(CANConstants.INTAKE_ID, MotorType.kBrushless);
    private final PIDController m_flexPIDController = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
    private final DutyCycleEncoder m_angleEncoder = new DutyCycleEncoder(CANConstants.WRIST_ANGLE_ENCODER);

    private ArmFeedforward m_feedForward = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV); 
    private Arm m_arm;


    public Intake(Arm arm) {
        super();
        m_intakeMotor.restoreFactoryDefaults();
        m_flexMotor.restoreFactoryDefaults();
        m_flexMotor.setInverted(true);
        m_angleEncoder.setPositionOffset(IntakeConstants.WRIST_OFFSET_DEGREES);
        m_arm = arm;


    } 

    public double getAngleRadians() {
        return Units.degreesToRadians((m_angleEncoder.getAbsolutePosition()) - (m_angleEncoder.getPositionOffset()));
    }

    public void pickUpCone() {
        m_intakeMotor.set(-IntakeConstants.INTAKE_SPEED);
    }

    public void pickUpCube() {
        m_intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    }

    public void stop() {
        m_intakeMotor.set(0);
    }
    
    public void flexClosedLoop(double velocity) {
        double feedForward = m_feedForward.calculate(getGroundRelativeWristPossitionRadians() + Math.PI/2 ,velocity); //calculate feed forward
        double percentOutput = 0;//calculate percent out from feed forward
        m_flexMotor.set(percentOutput);
        m_flexMotor.set(RobotContainer.voltageToPercentOutput(feedForward));
    }

    public double getGroundRelativeWristPossitionRadians(){
        return m_arm.getArmAngleRadians() + getAngleRadians();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("WristAngle",m_angleEncoder.getAbsolutePosition());
        /*  because the flex motor is so fast commented out until we get good PID values and limits 
        so we dont break the wrist 
        m_flexPIDController.setTolerance(IntakeConstants.WRIST_TOLERANCE.getRadians());
        m_flexPIDController.setSetpoint(IntakeConstants.TARGET_WRIST_ANGLE.getRadians());
        if (m_flexPIDController.atSetpoint()){
            flexClosedLoop(0);
        } else {
            flexClosedLoop(m_flexPIDController.calculate(m_angleEncoder.get()));
        }
        */
    }
    
    }


