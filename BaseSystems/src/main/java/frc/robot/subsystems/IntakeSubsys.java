// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsys extends SubsystemBase {
	// private static final DigitalInput intakeLimitSwitch = new DigitalInput(9);
	public final TalonFX indexerMotor =  new TalonFX(0);

	public IntakeSubsys() {
		indexerMotor.getConfigurator().apply(Constants.Configs.INTAKE_CONFIG);
	}

	public void setSpeed(IntakeSpeed speed) {
		indexerMotor.set(speed.value);
	}
	
	// Called every ~20ms
	@Override
	public void periodic() {}

	public enum IntakeSpeed {
		NONE(0),
		INTAKE(0.5),
		EJECT(-0.5);

		public final double value;
		IntakeSpeed(double value) {
			this.value = value;
		}
	}
}