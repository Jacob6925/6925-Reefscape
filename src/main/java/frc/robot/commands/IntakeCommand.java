// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PipeIntakeSubsys;
import frc.robot.subsystems.PipeIntakeSubsys.PipeIntakeSpeed;

public class IntakeCommand extends Command {
  private final PipeIntakeSubsys pippeIntakeSubsys;
  private final PipeIntakeSpeed speed;

  public IntakeCommand(PipeIntakeSubsys pipeIntakeSubsys, PipeIntakeSpeed speed) {
    addRequirements(pipeIntakeSubsys);

    this.pippeIntakeSubsys = pipeIntakeSubsys;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    pippeIntakeSubsys.setSpeedOfPipeIntake(speed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    pippeIntakeSubsys.setSpeedOfPipeIntake(PipeIntakeSpeed.OFF);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
