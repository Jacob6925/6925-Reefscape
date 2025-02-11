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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsys extends SubsystemBase {
  public final TalonFX pivotMotor = new TalonFX(8);
  private final TalonFX otherPivotMotor = new TalonFX(9);

  private final ProfiledPIDController pidController;

  public WristSubsys() {
    pivotMotor.getConfigurator().apply(Constants.Configs.WRIST_CONFIG);
    otherPivotMotor.setControl(new Follower(pivotMotor.getDeviceID(), true));

    pidController = new ProfiledPIDController(
      Constants.WristConstants.kP,
      Constants.WristConstants.kI,
      Constants.WristConstants.kD,
      new TrapezoidProfile.Constraints(Constants.WristConstants.MAX_VELOCITY, Constants.WristConstants.MAX_ACCEL)
    );
  }

  public void goTo(IntakePivotSetpoint setpoint) {
    pidController.setGoal(setpoint.rotations);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("rotations", pivotMotor.getPosition().getValueAsDouble());

    if (!DriverStation.isEnabled()) {
      pivotMotor.set(0);
      return;
    }

    double output = pidController.calculate(pivotMotor.getPosition().getValueAsDouble());
    output = MathUtil.clamp(output, -1, 1);
    pivotMotor.set(output);
  }

  public enum IntakePivotSetpoint {
    MAX_HEIGHT(Constants.WristConstants.MAX_HEIGHT),
    MIN_HEIGHT(0.0);

    public final double rotations;
    private IntakePivotSetpoint(double rotations) {
      this.rotations = rotations;
    }
  }
}
