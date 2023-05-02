package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.constants.DrivetrainConstants.*;


/**
 * Move to Position
 */
public class DriveToPosePID extends CommandBase {

    public Drivetrain swerve;
    public Pose2d pose2d;
    public Supplier<Pose2d> pose2dSupplier;

    HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        new PIDController(TranslationGains.kP, TranslationGains.kI,TranslationGains.kD),
        new PIDController(TranslationGains.kP, TranslationGains.kI,TranslationGains.kD),
        new ProfiledPIDController(ThetaGains.kP, ThetaGains.kI,ThetaGains.kD,
            new TrapezoidProfile.Constraints(maxAngularVelocity,
                Math.PI)));



    /**
     * Move to Position
     *
     * @param swerve Swerve Drive Subsystem
     * @param pose2dSupplier Supplier of Pose2d
     */
    public DriveToPosePID(Drivetrain swerve, Supplier<Pose2d> pose2dSupplier) {
        super();
        this.swerve = swerve;
        this.pose2dSupplier = pose2dSupplier;
        this.addRequirements(swerve);
        holonomicDriveController.setTolerance(new Pose2d(.04, .04, Rotation2d.fromDegrees(3)));
        
    }



    @Override
    public void initialize() {
        pose2d = pose2dSupplier.get();

    }

    @Override
    public void execute() {
        ChassisSpeeds ctrlEffort =
            holonomicDriveController.calculate(swerve.getPose(), pose2d, 0, pose2d.getRotation());
        swerve.driveClosedLoop(ctrlEffort);
        System.out.println("Pose Error" + (pose2d.getX() - swerve.getPose().getX()) + "," + (pose2d.getY() - swerve.getPose().getY()));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0,0,0);
    }

    @Override
    public boolean isFinished() {
        return holonomicDriveController.atReference();
    }
}
