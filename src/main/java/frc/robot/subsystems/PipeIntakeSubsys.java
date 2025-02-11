// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PipeIntakeSubsys extends SubsystemBase {
	private final TalonFX pipeIntake = new TalonFX(11);

	public PipeIntakeSubsys() {
		pipeIntake.getConfigurator().apply(Constants.Configs.PIPE_INTAKE_CONFIG);
	}

	public void setSpeedOfPipeIntake(PipeIntakeSpeed speed) {
		pipeIntake.set(speed.value);
	}
	
	// Called every ~20ms
	@Override
	public void periodic() {}

	public enum PipeIntakeSpeed {
		OFF(0),
		INTAKE(0.5),
		EJECT(-0.5);

		public final double value;
		PipeIntakeSpeed(double value) {
			this.value = value;
		}
	}
}