// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsys;
import frc.robot.subsystems.IntakeSubsys.IntakeSpeed;

public class IntakeCommand extends Command {
  private final IntakeSubsys intakeSubsys;
  private final IntakeSpeed speed;

  public IntakeCommand(IntakeSubsys intakeSubsys, IntakeSpeed speed) {
    addRequirements(intakeSubsys);

    this.intakeSubsys = intakeSubsys;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    intakeSubsys.setSpeed(speed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    intakeSubsys.setSpeed(IntakeSpeed.OFF);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
