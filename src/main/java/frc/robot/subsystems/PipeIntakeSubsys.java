// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallIntakeSubsys.BallIntakeSpeed;

public class PipeIntakeSubsys extends SubsystemBase {
	private final TalonFX pipeIntake = new TalonFX(11);

	public PipeIntakeSubsys() {
		pipeIntake.getConfigurator().apply(Constants.Configs.PIPE_INTAKE_CONFIG);
	}

	public void setSpeed(PipeIntakeSpeed speed) {
		pipeIntake.set(speed.value);
	}

	public Command setSpeedCommand(PipeIntakeSpeed speed) {
		return Commands.runOnce(() -> pipeIntake.set(speed.value), this);
	}

	public void setSpeedFromBallSpeed(BallIntakeSpeed speed) {
		pipeIntake.set(speed.value);
	}
	
	// Called every ~20ms
	@Override
	public void periodic() {}

	public enum PipeIntakeSpeed {
		OFF(0),
		INTAKE(0.4),
		EJECT(-0.5),
		L1_EJECT(-0.25);

		public final double value;
		PipeIntakeSpeed(double value) {
			this.value = value;
		}
	}
}