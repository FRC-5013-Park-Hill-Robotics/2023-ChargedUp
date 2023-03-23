// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.LimeLightConstants;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.constants.DrivetrainConstants.ThetaGains;
import frc.robot.subsystems.LimeLight;

public class AllignOnGamePiece extends CommandBase {
  /** Creates a new AllignOnGamePiece. */
  private LimeLight m_LimeLight;
  private Drivetrain m_Drivetrain;
  private PIDController thetaController = new PIDController(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD);
  public AllignOnGamePiece(Drivetrain drivetrain, LimeLight limelight) {
    addRequirements(drivetrain);
    m_Drivetrain = drivetrain;
    m_LimeLight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
    thetaController.setTolerance(LimeLightConstants.ALLIGNMENT_TOLLERANCE_RADIANS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thetaOutput = 0;
    double xOutput = 0;
    double yOutput = 0;
		if (m_LimeLight.hasTarget()){
			double vertical_angle = m_LimeLight.getVerticalAngleOfErrorDegrees();
			double horizontal_amgle = -m_LimeLight.getHorizontalAngleOfErrorDegrees() ;
			double setpoint = Math.toRadians(horizontal_amgle)+ m_Drivetrain.getYawR2d().getRadians();
      thetaController.setSetpoint(setpoint);

			if (!thetaController.atSetpoint() ){
				thetaOutput = thetaController.calculate(m_Drivetrain.getYawR2d().getRadians(), setpoint);
			} else {

      }
		} else {
			System.out.println("NO TARGET");
		}
    m_Drivetrain.drive(xOutput, yOutput, thetaOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
