// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsys extends SubsystemBase {
  private final TalonFX elevatorMotor = new TalonFX(3);
  private final TalonFX secondElevatorMotor = new TalonFX(6);
  private final Follower follower = new Follower(elevatorMotor.getDeviceID(), true);

  public ElevatorSubsys() {
    elevatorMotor.getConfigurator().apply(Constants.Configs.ELEVATOR_CONFIG);
    secondElevatorMotor.setControl(follower);
  }

  public void setSpeed(ElevatorSpeed speed) {
    elevatorMotor.set(speed.value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum ElevatorSpeed {
    OFF(0),
    UP(0.5),
    DOWN(-0.5);

    public final double value;
    private ElevatorSpeed(double value) {
      this.value = value;
    }
  }
}