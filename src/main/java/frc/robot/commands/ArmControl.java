// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LogitechController;
import frc.robot.subsystems.Arm;
import frc.robot.constants.GlobalConstants.ControllerConstants;

public class ArmControl extends CommandBase {
  /** Creates a new ArmControl. */
  private LogitechController m_gamepad;
  private Arm m_arm;

  public ArmControl(Arm arm, LogitechController gamepad) {
    addRequirements(arm);
    m_gamepad = gamepad;
    m_arm = arm; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotatePercent = modifyAxis(m_gamepad.getLeftY());
    double extendPercent = modifyAxis(m_gamepad.getRightY());
    m_arm.extend(-extendPercent);
    m_arm.rotate(-rotatePercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  
  private static double modifyAxis(double value) {
	
		return modifyAxis(value, 1);
	}

	private static double modifyAxis(double value, int exponent) {
		// Deadband
		value = MathUtil.applyDeadband(value, ControllerConstants.DEADBAND);

		value = Math.copySign(Math.pow(value, exponent), value);

		return value;
	}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
