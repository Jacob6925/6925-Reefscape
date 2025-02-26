// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  private final ProfiledPIDController pidControllerUp;
  private final ProfiledPIDController pidControllerDown;
  private final ElevatorFeedforward feedforward;

  public ElevatorSubsys() {
    elevatorMotor.getConfigurator().apply(Constants.Configs.ELEVATOR_CONFIG);
    secondElevatorMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));

    pidControllerUp = new ProfiledPIDController(
      ElevatorConstants.kP_Up,
      ElevatorConstants.kI_Up,
      ElevatorConstants.kD_Up,
      new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY_UP, ElevatorConstants.MAX_ACCEL_UP)
    );

    pidControllerDown = new ProfiledPIDController(
      ElevatorConstants.kP_Down,
      ElevatorConstants.kI_Down,
      ElevatorConstants.kD_Down,
      new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY_DOWN, ElevatorConstants.MAX_ACCEL_DOWN)
    );

    feedforward = new ElevatorFeedforward(
      ElevatorConstants.kS,
      ElevatorConstants.kG,
      ElevatorConstants.kV
    );

    pidControllerUp.reset(0);
    pidControllerDown.reset(0);
  }

  public Command goTo(ElevatorPosition setpoint) {
    return Commands.runOnce(() -> {
      if (setpoint.rotations > elevatorMotor.getPosition().getValueAsDouble()) {
        // need to go up
        pidControllerUp.setGoal(setpoint.rotations);
        pidControllerDown.setGoal(null);
      } else {
        // need to go down
        pidControllerUp.setGoal(null);
        pidControllerDown.setGoal(setpoint.rotations);
      }
    });
  }

  private double pidOutput = 0;
  private double feedForwardOutput = 0;

  @Override
  public void periodic() {
    SmartDashboard.putNumber("EPos0", elevatorMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("EPos1", secondElevatorMotor.getPosition().getValueAsDouble());

    double elevatorPosition = elevatorMotor.getPosition().getValueAsDouble();
    if (pidControllerUp.getGoal() != null) {
      pidOutput = pidControllerUp.calculate(elevatorPosition);
      feedForwardOutput = feedforward.calculate(pidControllerUp.getSetpoint().velocity);
      SmartDashboard.putString("CurrPID", "Up");
    } else if (pidControllerDown.getGoal() != null) {
      pidOutput = pidControllerDown.calculate(elevatorPosition);
      feedForwardOutput = feedforward.calculate(pidControllerDown.getSetpoint().velocity);
      SmartDashboard.putString("CurrPID", "Down");
    } else {
      pidOutput = 0;
      feedForwardOutput = feedforward.calculate(0);
    }

    elevatorMotor.setVoltage(pidOutput + feedForwardOutput);

    SmartDashboard.putNumber("PID Output", pidOutput);
    SmartDashboard.putNumber("FF Output", feedForwardOutput);

    maxes.checkMaxVel(elevatorMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elev Max Vel", maxes.getMaxVel());
  }

  public enum ElevatorPosition {
    MAX_HEIGHT(Constants.Configs.ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold),
    HALF_HEIGHT(Constants.Configs.ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold/2.0),
    QUARTER_HEIGHT(Constants.Configs.ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold/4.0),
    THREE_ROT(3),
    MIN_HEIGHT(0);

    public final double rotations;
    private ElevatorPosition(double rotations) {
      this.rotations = rotations;
    }
  }
}