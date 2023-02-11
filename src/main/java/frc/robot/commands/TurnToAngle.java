package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.constants.DrivetrainConstants.ThetaGains.*;

public class TurnToAngle extends PIDCommand {
    public TurnToAngle(double targetAngleDegrees, Drivetrain drive) {
        super(
            new PIDController(DrivetrainConstants.driveKP, DrivetrainConstants.driveKI, DrivetrainConstants.driveKD),

            drive::getHeadingRadians,

            targetAngleDegrees,

            output -> drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, output, drive.getYawR2d())));
            getController().setTolerance(kTurnToleranceRad,kTurnRateToleranceRadPerS);
      }
 @Override
 public boolean isFinished(){
     return getController().atSetpoint() ;
 }
   

}