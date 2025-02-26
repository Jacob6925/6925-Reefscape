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
//import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsys extends SubsystemBase {
  private final TalonFX elevatorMotor = new TalonFX(12);
  private final TalonFX secondElevatorMotor = new TalonFX(13);

  private final ProfiledPIDController pidController;
  private final ElevatorFeedforward feedforward;

  public ElevatorSubsys() {
    elevatorMotor.getConfigurator().apply(Constants.Configs.ELEVATOR_CONFIG);
    secondElevatorMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));

    pidController = new ProfiledPIDController(
      ElevatorConstants.kP_Up,
      ElevatorConstants.kI_Up,
      ElevatorConstants.kD_Up,
      new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY_UP, ElevatorConstants.MAX_ACCEL_UP)
    );
    // pidController = new ProfiledPIDController(0,0,0,
    //   new TrapezoidProfile.Constraints(Constants.ElevatorConstants.MAX_VELOCITY_UP, Constants.ElevatorConstants.MAX_ACCEL_UP)
    // );

    feedforward = new ElevatorFeedforward(
      ElevatorConstants.kS,
      ElevatorConstants.kG,
      ElevatorConstants.kV
    );

    pidController.reset(0);
  }

  public Command goTo(ElevatorPosition setpoint) {
    return Commands.runOnce(() -> {
      pidController.setGoal(setpoint.rotations);
    });
  }

  private double pidOutput = 0;
  private double feedForwardOutput = 0;

  @Override
  public void periodic() {
    SmartDashboard.putNumber("EPos0", elevatorMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("EPos1", secondElevatorMotor.getPosition().getValueAsDouble());

    // double elevatorPosition = elevatorMotor.getPosition().getValueAsDouble();
    // if (pidControllerUp.getGoal() != null) {
    //   pidOutput = pidControllerUp.calculate(elevatorPosition);
    //   feedForwardOutput = feedforward.calculate(pidControllerUp.getSetpoint().velocity);
    //   SmartDashboard.putString("CurrPID", "Up");
    // } else if (pidControllerDown.getGoal() != null) {
    //   pidOutput = pidControllerDown.calculate(elevatorPosition);
    //   feedForwardOutput = feedforward.calculate(pidControllerDown.getSetpoint().velocity);
    //   SmartDashboard.putString("CurrPID", "Down");
    // } else {
    //   pidOutput = 0;
    //   feedForwardOutput = feedforward.calculate(0);
    // }

    pidOutput = pidController.calculate(elevatorMotor.getPosition().getValueAsDouble());
    // feedForwardOutput = feedforward.calculate(pidControllerUp.getSetpoint().velocity);
    feedForwardOutput = 0;
    elevatorMotor.setVoltage(pidOutput + feedForwardOutput);

    SmartDashboard.putNumber("PID Output", pidOutput);
    SmartDashboard.putNumber("FF Output", feedForwardOutput);
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