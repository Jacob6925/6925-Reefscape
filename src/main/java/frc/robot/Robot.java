// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ElevatorSubsys;

public class Robot extends TimedRobot {
  private static Robot instance;
  private Command m_autonomousCommand;

  // private final FlightSimSwerveRobotContainer m_robotContainer;
  // private final TestingRobotContainer m_robotContainer;
  // private final SimulationRobotContainer m_robotContainer;
  private final RobotContainer m_robotContainer;

  public Robot() {
    // m_robotContainer = new FlightSimSwerveRobotContainer();
    // m_robotContainer = new TestingRobotContainer();
    // m_robotContainer = new SimulationRobotContainer();
    m_robotContainer = new RobotContainer();
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    if (RobotContainer.getInstance() != null && RobotContainer.getInstance().drivetrain != null) RobotContainer.getInstance().drivetrain.manualAlign();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
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
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    ElevatorSubsys instance = ElevatorSubsys.getInstance();
    if (instance != null) ElevatorSubsys.getInstance().resetElevatorSetpoint();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
