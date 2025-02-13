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
  public final TalonFX wristMotor = new TalonFX(8);
  private final TalonFX wristFollowerMotor = new TalonFX(9);

  private final ProfiledPIDController pidController;

  public WristSubsys() {
    wristMotor.getConfigurator().apply(Constants.Configs.WRIST_CONFIG);
    wristFollowerMotor.setControl(new Follower(wristMotor.getDeviceID(), true));

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
    SmartDashboard.putNumber("rotations", wristMotor.getPosition().getValueAsDouble());

    if (!DriverStation.isEnabled()) {
      wristMotor.set(0);
      return;
    }

    double output = pidController.calculate(wristMotor.getPosition().getValueAsDouble());
    output = MathUtil.clamp(output, -1, 1);
    wristMotor.set(output);
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
