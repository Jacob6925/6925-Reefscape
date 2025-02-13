// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.positiongetter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GetPositionSubsys extends SubsystemBase {
  public final TalonFX motor;
  private final TalonFX motor2;

  public GetPositionSubsys(int deviceId0, int deviceId1) {
    motor = new TalonFX(deviceId0);
    motor2 = new TalonFX(deviceId1);
    motor2.setControl(new Follower(deviceId0, false));

    TalonFXConfiguration config = new TalonFXConfiguration();
    // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Main Motor Rotations", motor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Sub-Motor Rotations", motor2.getPosition().getValueAsDouble());
  }
}
