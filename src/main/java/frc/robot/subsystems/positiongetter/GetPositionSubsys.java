// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.positiongetter;

import java.util.function.BiConsumer;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GetPositionSubsys extends SubsystemBase {
  private final String id;
  public final TalonFX motor;
  private final TalonFX motor2;

  public GetPositionSubsys(String id, int id0, int id1, boolean oppositeFollowerDirection, BiConsumer<GetPositionSubsys, TalonFX> settings) {
    motor = new TalonFX(id0);
    motor2 = new TalonFX(id1);
    motor2.setControl(new Follower(motor.getDeviceID(), oppositeFollowerDirection));

    settings.accept(this, motor);
    this.id = id;
  }
  public GetPositionSubsys(String id, int id0, BiConsumer<GetPositionSubsys, TalonFX> settings) {
    motor = new TalonFX(id0);
    motor2 = null;
    
    settings.accept(this, motor);
    this.id = id;
  }

  public Command setSpeed(int speed) {
    return Commands.runOnce(() -> motor.set(speed), this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("(" + id + ") Main Motor Rotations", motor.getPosition().getValueAsDouble());
    if (motor2 != null) SmartDashboard.putNumber("(" + id + ") Sub-Motor Rotations", motor2.getPosition().getValueAsDouble());
  }
}
