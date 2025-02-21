// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

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
		ballIntake.set(speed.value);
	}
	
	// Called every ~20ms
	@Override
	public void periodic() {}

	public enum BallIntakeSpeed {
		OFF(0),
		INTAKE(0.5),
		EJECT(-0.5);

		public final double value;
		BallIntakeSpeed(double value) {
			this.value = value;
		}
	}
}