// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class CTREConfigs {
    public final TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
    public final TalonFXConfiguration INTAKE_CONFIG = new TalonFXConfiguration();

    public CTREConfigs() {
        // Elevator Config
        ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // CHANGE TO TRUE ONCE LIMIT FOUND
        ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0; // set limit
        ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        // Intake Config
    }
}