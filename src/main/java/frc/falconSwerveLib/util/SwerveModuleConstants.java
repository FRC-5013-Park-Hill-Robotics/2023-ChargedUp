package frc.falconSwerveLib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final CanPort driveMotorID;
    public final CanPort angleMotorID;
    public final CanPort cancoderID;
    public final Rotation2d angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(CanPort driveMotorID, CanPort angleMotorID, CanPort canCoderID, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}
