// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsys extends SubsystemBase {
  private static ElevatorSubsys instance;

  private final TalonFX elevatorMotor = new TalonFX(12);
  private final TalonFX secondElevatorMotor = new TalonFX(13);

  private final ProfiledPIDController pidController;
  // private final ElevatorFeedforward feedforward;
  
  private double pidOutput = 0;
  private double feedForwardOutput = 0;
  private boolean controlBySpeed = false;

  public ElevatorSubsys() {
    if (instance != null) throw new Error("Elevator already instantiated!");
    instance = this;

    elevatorMotor.getConfigurator().apply(Constants.Configs.ELEVATOR_CONFIG);
    secondElevatorMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));

    pidController = new ProfiledPIDController(
      ElevatorConstants.kP,
      ElevatorConstants.kI,
      ElevatorConstants.kD,
      new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCEL)
    );

    // feedforward = new ElevatorFeedforward(
    //   ElevatorConstants.kS,
    //   ElevatorConstants.kG,
    //   ElevatorConstants.kV
    // );

    pidController.reset(0);
  }

  public Command goTo(ElevatorPosition setpoint) {
    return Commands.runOnce(() -> {
      pidController.setGoal(setpoint.rotations);
    });
  }

  public Command setSpeedCommand(double speed) {
    return new InstantCommand(() -> {
      controlBySpeed = true;
      elevatorMotor.set(speed);
    }, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ELEV POSITION", elevatorMotor.getPosition().getValueAsDouble());

    if (controlBySpeed) return;
    
    double output;
    if (pidController.getGoal().position == ElevatorPosition.MIN_HEIGHT.rotations && getMotorRotations() < 1 ) {
      output = 0;
    } else {
      pidOutput = pidController.calculate(elevatorMotor.getPosition().getValueAsDouble());
      // feedForwardOutput = feedforward.calculate(pidControllerUp.getSetpoint().velocity);
      feedForwardOutput = 0;
      output = pidOutput + feedForwardOutput;
    }
    elevatorMotor.setVoltage(output);
  }

  public enum ElevatorPosition {
    MAX_HEIGHT(Constants.Configs.ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold),
    HALF_HEIGHT(Constants.Configs.ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold/2.0),
    QUARTER_HEIGHT(Constants.Configs.ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold/4.0),
    THREE_ROT(3),
    MIN_HEIGHT(0),
    
    L1(1.678),
    L2(9),
    L3(16.35),
    L4(MAX_HEIGHT.rotations),
    HUMAN_PLAYER_INTAKE(2.075), //2.1 a bit too high
    REMOVE_ALGAE_L2(L2.rotations + 1),
    REMOVE_ALGAE_L3(L3.rotations + 1),
    DEPOSIT_BALL(2),
    
    BALL_HOLD(3);

    public final double rotations;
    private ElevatorPosition(double rotations) {
      this.rotations = rotations;
    }
  }

  public static ElevatorSubsys getInstance() {
    return instance;
  }

  public void resetElevatorSetpoint() {
    elevatorMotor.set(0);
    controlBySpeed = false;
    pidController.setGoal(new TrapezoidProfile.State());
    elevatorMotor.setPosition(0);
    secondElevatorMotor.setPosition(0);
  }

  public double getMotorRotations() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public ProfiledPIDController getPIDController() {
    return pidController;
  }
}