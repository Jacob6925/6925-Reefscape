// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallIntakeSubsys;
import frc.robot.subsystems.BallIntakeSubsys.BallIntakeSpeed;

public class BallIntakeCommand extends Command {
  private final BallIntakeSubsys ballIntakeSubsys;
  private final BallIntakeSpeed speed;

  public BallIntakeCommand(BallIntakeSubsys ballIntakeSubsys, BallIntakeSpeed speed) {
    addRequirements(ballIntakeSubsys);

    this.ballIntakeSubsys = ballIntakeSubsys;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    ballIntakeSubsys.setSpeedOfBallIntake(speed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    ballIntakeSubsys.setSpeedOfBallIntake(BallIntakeSpeed.OFF);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
