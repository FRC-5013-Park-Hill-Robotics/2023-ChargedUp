// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmRotate extends CommandBase {
  private Arm m_arm;
  private double m_angleRadians;
  /** Creates a new ArmRotate. */
  public ArmRotate(Arm arm, double  angleRadians) {
    addRequirements(arm);
    m_arm = arm;
    m_angleRadians = angleRadians;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.getRotationPIDController().reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PIDController rotationController = m_arm.getRotationPIDController();
    rotationController.setSetpoint(m_angleRadians);
    m_arm.rotateClosedLoop(rotationController.calculate(m_arm.getArmAngleRadians()));
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.getRotationPIDController().atSetpoint();
  }
}
