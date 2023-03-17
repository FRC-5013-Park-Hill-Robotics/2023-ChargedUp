// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.GlobalConstants.IntakeConstants;
import frc.robot.trobot5013lib.RevThroughBoreEncoder;
import frc.robot.RobotContainer;


/** Add your docs here. */
public class Intake extends SubsystemBase {
    private final CANSparkMax m_flexMotor = new CANSparkMax(CANConstants.WRIST_ID, MotorType.kBrushless);
    private final CANSparkMax m_intakeMotor = new CANSparkMax(CANConstants.INTAKE_ID, MotorType.kBrushless);
    private final PIDController m_flexPIDController = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
    private final RevThroughBoreEncoder m_angleEncoder = new RevThroughBoreEncoder(CANConstants.WRIST_ANGLE_ENCODER);

    private ArmFeedforward m_feedForward = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV); 
    private Arm m_arm;


    public Intake(Arm arm) {
        super();
        m_intakeMotor.restoreFactoryDefaults();
        m_flexMotor.restoreFactoryDefaults();
        m_flexMotor.setInverted(true);
        m_flexMotor.setIdleMode(IdleMode.kBrake);
        m_flexPIDController.enableContinuousInput(0, 2 * Math.PI);
        m_angleEncoder.setOffset(IntakeConstants.WRIST_OFFSET_DEGREES);
        m_angleEncoder.setInverted(true);
        m_arm = arm;


    } 

    public double getAngleRadians() {
        return ((m_angleEncoder.getAngle()).getRadians());
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
        SmartDashboard.putNumber("Flex velocity", velocity);
        double feedForward = m_feedForward.calculate(getGroundRelativeWristPossitionRadians() ,velocity); //calculate feed forward
        m_flexMotor.setVoltage(feedForward);
    }

    public double getGroundRelativeWristPossitionRadians(){
        return m_arm.getArmAngleRadians() + getAngleRadians();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("WristAngle",(m_angleEncoder.getAngle()).getDegrees());
        SmartDashboard.putNumber("WriseAngleGround",Units.radiansToDegrees(getGroundRelativeWristPossitionRadians())); 
        if (m_arm.getArmAngleRadians() < Units.degreesToRadians(85) || m_arm.getArmAngleRadians() > Units.degreesToRadians(337) ){
            m_flexPIDController.setTolerance(IntakeConstants.WRIST_TOLERANCE.getRadians());
            m_flexPIDController.setSetpoint(IntakeConstants.TARGET_WRIST_ANGLE.getRadians());
            flexClosedLoop(m_flexPIDController.calculate(getGroundRelativeWristPossitionRadians()));
        }
    }
    
 }


