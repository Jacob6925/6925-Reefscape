// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.PipeIntakeSubsys.PipeIntakeSpeed;

public class BallIntakeSubsys extends SubsystemBase {
  private final TalonFX ballIntake = new TalonFX(10);

	public BallIntakeSubsys() {
		ballIntake.getConfigurator().apply(Constants.Configs.BALL_INTAKE_CONFIG);
	}

	public void setSpeed(BallIntakeSpeed speed) {
		ballIntake.set(speed.value);
	}

	public void setSpeedFromPipeSpeed(PipeIntakeSpeed speed) {
		ballIntake.set(-speed.value);
	}

	public Command setSpeedCommand(BallIntakeSpeed speed) {
		return Commands.runOnce(() -> ballIntake.set(speed.value), this);
	}

	public Command setSpeedFromPipeSpeedCommand(PipeIntakeSpeed speed) {
		return Commands.runOnce(() -> ballIntake.set(-speed.value), this);
	}
	
	// Called every ~20ms
	@Override
	public void periodic() {}

	public enum BallIntakeSpeed {
		OFF(0),
		INTAKE(0.5),
		EJECT(-0.5),
		HOLD_BALL(0.1),
		OUTTAKE_MAX(-1);

		public final double value;
		BallIntakeSpeed(double value) {
			this.value = value;
		}
	}
}