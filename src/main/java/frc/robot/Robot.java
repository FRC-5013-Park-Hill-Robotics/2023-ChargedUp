// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PhotonVision;

public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private Command m_autonomousCommand;
  private Alliance m_alliance = Alliance.Invalid;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    m_robotContainer = new RobotContainer();
    m_robotContainer.getArm().hold();
    PathPlannerCommandFactory.init();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  private void checkUpdateAlliance(){
    
    Alliance alliance =  DriverStation.getAlliance();
      if(DriverStation.isDSAttached() && alliance  != Alliance.Invalid && alliance != m_alliance){
        PhotonVision pv = m_robotContainer.getPhotonVision();
        if (pv!= null){
          pv.initialize();
        }
        LimeLight frontLL = m_robotContainer.getFrontLimelight();
        LimeLight backLL = m_robotContainer.getBackLimelight();
        frontLL.setAlliance(alliance);
        backLL.setAlliance(alliance);
      }
  }
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    checkUpdateAlliance();
    RobotContainer.getInstance().getDrivetrain().resetModulesToAbsolute();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    checkUpdateAlliance();
  }

  @Override
  public void teleopPeriodic() {
    checkUpdateAlliance();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
