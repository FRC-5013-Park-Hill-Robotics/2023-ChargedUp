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
  private boolean hasExtended = false;
  private boolean hasRotated = false;
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
    hasExtended = false;
    hasRotated = false;
    m_arm.getRotationPIDController().reset();
    m_arm.getExtensionPIDController().reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PIDController extensPidController = m_arm.getExtensionPIDController();
    extensPidController.setSetpoint(m_length);
    double extend = extensPidController.calculate(m_arm.getCurrentExtensionDistance());
    m_arm.extendClosedLoop( extend );

    PIDController rotationController = m_arm.getRotationPIDController();
    rotationController.setSetpoint(m_angleRadians);
    double rotate = rotationController.calculate(m_arm.getArmAngleRadians());
    m_arm.rotateClosedLoop(rotate);
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
    if (m_arm.getRotationPIDController().atSetpoint()) {
      hasRotated =true;
    } 

    if (m_arm.getExtensionPIDController().atSetpoint()) {
      hasExtended = true;
    }
    return hasRotated && hasExtended;
  }
}
