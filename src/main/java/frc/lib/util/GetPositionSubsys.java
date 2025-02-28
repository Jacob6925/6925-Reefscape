// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.function.Consumer;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GetPositionSubsys extends SubsystemBase {
  private final String id;
  public final TalonFX mainMotor;
  public final TalonFX subMotor;

  public final MaxCollections maxes;

  public GetPositionSubsys(String id, int id0, int id1, boolean oppositeFollowerDirection, Consumer<GetPositionSubsys> settings) {
    mainMotor = new TalonFX(id0);
    if (id1 >= 0) {
      subMotor = new TalonFX(id1);
      subMotor.setControl(new Follower(mainMotor.getDeviceID(), oppositeFollowerDirection));
    } else {
      subMotor = null;
    }

    this.id = id;
    this.maxes = new MaxCollections(id, () -> mainMotor.getVelocity().getValueAsDouble(), () -> mainMotor.getAcceleration().getValueAsDouble());
    
    settings.accept(this);
  }

  public GetPositionSubsys(String id, int id0, Consumer<GetPositionSubsys> settings) {
    this(id, id0, -1, false, settings);
  }

  public Command setSpeed(int speed) {
    return Commands.runOnce(() -> mainMotor.set(speed), this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("(" + id + ") Main Motor Rotations", mainMotor.getPosition().getValueAsDouble());
    if (subMotor != null) SmartDashboard.putNumber("(" + id + ") Sub-Motor Rotations", subMotor.getPosition().getValueAsDouble());
  }
}
