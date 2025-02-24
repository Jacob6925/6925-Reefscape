// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.MaxFinder;
//import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsys extends SubsystemBase {
  private final TalonFX elevatorMotor = new TalonFX(12);
  private final TalonFX secondElevatorMotor = new TalonFX(13);

  MaxFinder maxes = new MaxFinder();

  private final ProfiledPIDController pidController;
  private final ElevatorFeedforward feedforward;

  public ElevatorSubsys() {
    elevatorMotor.getConfigurator().apply(Constants.Configs.ELEVATOR_CONFIG);
    secondElevatorMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));

    pidController = new ProfiledPIDController(
      ElevatorConstants.kP,
      ElevatorConstants.kI,
      ElevatorConstants.kD,
      new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCEL)
    );

    feedforward = new ElevatorFeedforward(
      ElevatorConstants.kS,
      ElevatorConstants.kG,
      ElevatorConstants.kV
    );

    pidController.reset(0);
  }

  public Command goTo(ElevatorPosition setpoint) {
    return Commands.runOnce(() -> pidController.setGoal(setpoint.rotations), this);
  }

  private double pidOutput = -999;
  private double feedForwardOutput = -999;

  @Override
  public void periodic() {
    if (!DriverStation.isEnabled()) {
      elevatorMotor.set(0);
      return;
    }

    pidOutput = pidController.calculate(elevatorMotor.getPosition().getValueAsDouble());
    if (pidOutput < 0) {
      feedForwardOutput = feedforward.calculate(pidController.getSetpoint().velocity);
    } else {
      feedForwardOutput = 0;
    }
    elevatorMotor.setVoltage(pidOutput + feedForwardOutput);

    // double out = pidController.calculate(elevatorMotor.getPosition().getValueAsDouble());
    // out = MathUtil.clamp(out, -1, 1);
    // elevatorMotor.set(out);

    SmartDashboard.putNumber("PID Output", pidOutput);
    SmartDashboard.putNumber("FF Output", feedForwardOutput);

    maxes.checkMaxVel(elevatorMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elev Max Vel", maxes.getMaxVel());
  }

  public enum ElevatorPosition {
    MAX_HEIGHT(ElevatorConstants.MAX_HEIGHT),
    HALF_HEIGHT(25/2.0),
    THREE_ROT(3),
    MIN_HEIGHT(0.0);

    public final double rotations;
    private ElevatorPosition(double rotations) {
      this.rotations = rotations;
    }
  }
}