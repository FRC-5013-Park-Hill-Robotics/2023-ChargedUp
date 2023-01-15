package com.swervedrivespecialties.swervelib;

public interface DriveController {
    void setReferenceVoltage(double voltage);
    double getStateDistance();
    double getStateVelocity();
    Object getDriveMotor();
}
