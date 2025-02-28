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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsys extends SubsystemBase {
  public final TalonFX wristMotor = new TalonFX(8);
  private final TalonFX wristFollowerMotor = new TalonFX(9);

  private final ProfiledPIDController pidController;
  private final ArmFeedforward feedForward;

  public WristSubsys() {
    wristMotor.getConfigurator().apply(Constants.Configs.WRIST_CONFIG);
    wristFollowerMotor.setControl(new Follower(wristMotor.getDeviceID(), true));

    pidController = new ProfiledPIDController(
      Constants.WristConstants.kP,
      Constants.WristConstants.kI,
      Constants.WristConstants.kD,
      new TrapezoidProfile.Constraints(Constants.WristConstants.MAX_VELOCITY, Constants.WristConstants.MAX_ACCEL)
    );

    feedForward = new ArmFeedforward(
      Constants.WristConstants.kS,
      Constants.WristConstants.kG,
      Constants.WristConstants.kV
    );
  }

  public void goTo(IntakePivotSetpoint setpoint) {
    pidController.setGoal(setpoint.rotations);
  }

  private double pidOutput = 0;
  private double feedForwardOutput = 0;

  @Override
  public void periodic() {
    double wristPosition = wristMotor.getPosition().getValueAsDouble();
    pidOutput = pidController.calculate(wristPosition);
    feedForwardOutput = feedForward.calculate(wristPosition, pidController.getSetpoint().velocity);
    
    wristMotor.setVoltage(pidOutput + feedForwardOutput);

    SmartDashboard.putNumber("PID Output", pidOutput);
    SmartDashboard.putNumber("FF Output", feedForwardOutput);
  }

  public enum IntakePivotSetpoint {
    MAX_POS(Constants.Configs.WRIST_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold),
    START_POS(0.0),
    MIN_POS(Constants.Configs.WRIST_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold);

    public final double rotations;
    private IntakePivotSetpoint(double rotations) {
      this.rotations = rotations;
    }
  }
}
