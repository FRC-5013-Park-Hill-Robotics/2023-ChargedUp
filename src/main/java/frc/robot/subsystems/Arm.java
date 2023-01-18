// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.swervedrivespecialties.swervelib.ctre.Falcon500DriveControllerFactoryBuilder;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;




/** Add your docs here. */
public class Arm extends SubsystemBase {
    private final TalonFX m_TalonFX = new TalonFX(0, getName());
    private final TalonFX m_rotationMotor = new TalonFX(0, getName());

    public void rotate(double percent) {

    }

    public void extendTo(double length) {

    }

    public void extend(double percent) {

    }

    public void rotateTo(Rotation2d angle) {

    }

}
