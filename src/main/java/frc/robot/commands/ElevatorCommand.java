// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ElevatorSubsys.ElevatorPosition;;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  private final ElevatorSubsys elevatorSubsys;
  private final ElevatorPosition position;
  
  public ElevatorCommand(ElevatorSubsys elevatorSubsys, ElevatorPosition position) {
    addRequirements(elevatorSubsys);
    this.elevatorSubsys = elevatorSubsys;
    this.position = position;
  }

  @Override
  public void initialize() {
    elevatorSubsys.goTo(position);
  }

  // Called every ~20ms
  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
