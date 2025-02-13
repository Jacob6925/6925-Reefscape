// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsys extends SubsystemBase {
  private final TalonFX elevatorMotor = new TalonFX(12);
  private final TalonFX secondElevatorMotor = new TalonFX(13);

  private final ProfiledPIDController pidController;

  public ElevatorSubsys() {
    elevatorMotor.getConfigurator().apply(Constants.Configs.ELEVATOR_CONFIG);
    secondElevatorMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));

    pidController = new ProfiledPIDController(
      ElevatorConstants.kP,
      ElevatorConstants.kI,
      ElevatorConstants.kD,
      new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCEL)
    );
  }

  public void goTo(ElevatorPosition setpoint) {
    pidController.setGoal(setpoint.meters);
  }

  @Override
  public void periodic() {
    if (!DriverStation.isEnabled()) {
      elevatorMotor.set(0);
      return;
    }

    double out = pidController.calculate(elevatorMotor.getPosition().getValueAsDouble());
    out = MathUtil.clamp(out, -1, 1);
    elevatorMotor.set(out);
  }

  public enum ElevatorPosition {
    MAX_HEIGHT(ElevatorConstants.MAX_HEIGHT),
    MIN_HEIGHT(0.0);

    public final double meters;
    private ElevatorPosition(double meters) {
      this.meters = meters;
    }
  }
}