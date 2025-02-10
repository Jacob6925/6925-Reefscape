// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.positiongetter;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GetPositionCommand extends Command {
  private final GetPositionSubsys getPositionSubsys;
  private final double speed;

  public GetPositionCommand(GetPositionSubsys getPositionSubsys, double speed) {
    addRequirements(getPositionSubsys);
    this.getPositionSubsys = getPositionSubsys;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    getPositionSubsys.motor.set(speed);
  }

  // every 20ms
  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    getPositionSubsys.motor.set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
