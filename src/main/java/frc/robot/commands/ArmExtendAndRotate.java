// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmExtendAndRotate extends CommandBase {
  private Arm m_arm;
  private double m_angleRadians;
  private double m_length;
  /** Creates a new ArmExtendAndRotate. */
  public ArmExtendAndRotate(Arm arm, double length, double angleRadians) {
    addRequirements(arm);
    m_arm = arm;
    m_angleRadians = angleRadians;
    m_length = length;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.getRotationPIDController().reset();
    m_arm.getExtensionPIDController().reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PIDController extensPidController = m_arm.getExtensionPIDController();
    extensPidController.setSetpoint(m_length);
    m_arm.extendClosedLoop(extensPidController.calculate(m_arm.getCurrentExtensionDistance()));
    PIDController rotationController = m_arm.getRotationPIDController();
    rotationController.setSetpoint(m_angleRadians);
    m_arm.rotateClosedLoop(rotationController.calculate(m_arm.getArmAngleRadians()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.hold();
    m_arm.holdExtension();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.getRotationPIDController().atSetpoint() && m_arm.getExtensionPIDController().atSetpoint();
  }
}
