// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsys extends SubsystemBase {
  public final TalonFX wristMotor = new TalonFX(8);
  private final TalonFX wristFollowerMotor = new TalonFX(9);

  private final ProfiledPIDController pidController;
  // private final ArmFeedforward feedForward;

  public WristSubsys() {
    wristMotor.getConfigurator().apply(Constants.Configs.WRIST_CONFIG);
    wristFollowerMotor.setControl(new Follower(wristMotor.getDeviceID(), true));

    pidController = new ProfiledPIDController(
      Constants.WristConstants.kP,
      Constants.WristConstants.kI,
      Constants.WristConstants.kD,
      new TrapezoidProfile.Constraints(Constants.WristConstants.MAX_VELOCITY, Constants.WristConstants.MAX_ACCEL)
    );

    // feedForward = new ArmFeedforward(
    //   Constants.WristConstants.kS,
    //   Constants.WristConstants.kG,
    //   Constants.WristConstants.kV
    // );
  }

  public Command goTo(WristSetpoint setpoint) {
    return Commands.runOnce(() -> pidController.setGoal(setpoint.rotations), this);
  }

  private double pidOutput = 0;
  private double feedForwardOutput = 0;

  @Override
  public void periodic() {
    double wristPosition = wristMotor.getPosition().getValueAsDouble();
    pidOutput = pidController.calculate(wristPosition);
    SmartDashboard.putNumber("CURR POSITION:", wristPosition);
    SmartDashboard.putNumber("GOAL:", pidController.getGoal().position);
    SmartDashboard.putNumber("OUTPUT:", pidOutput);

    // feedForwardOutput = feedForward.calculate(wristPosition, pidController.getSetpoint().velocity);
    feedForwardOutput = 0;
    
    wristMotor.setVoltage(pidOutput + feedForwardOutput);
  }

  public enum WristSetpoint {
    MAX_POS(Constants.Configs.WRIST_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold),
    START_POS(0),
    MIN_POS(Constants.Configs.WRIST_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold),
    HALF_FORWARD(Constants.Configs.WRIST_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold/2),
    HALF_REVERSE(Constants.Configs.WRIST_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold/2),
    
    HUMAN_PLAYER_INTAKE(6.75);

    public final double rotations;
    private WristSetpoint(double rotations) {
      this.rotations = rotations;
    }
  }
}
