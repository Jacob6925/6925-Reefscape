// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CTREConfigs {
    public final TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
    public final TalonFXConfiguration BALL_INTAKE_CONFIG = new TalonFXConfiguration();
    public final TalonFXConfiguration PIPE_INTAKE_CONFIG = new TalonFXConfiguration();
    public final TalonFXConfiguration WRIST_CONFIG = new TalonFXConfiguration();

    public CTREConfigs() {
        // Elevator Config
        ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // CHANGE TO TRUE ONCE LIMIT FOUND
        ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0; // set limit
        ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        ELEVATOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        ELEVATOR_CONFIG.CurrentLimits.StatorCurrentLimit = 50;

        ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 50;

        // Ball Intake Config
        BALL_INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        BALL_INTAKE_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        BALL_INTAKE_CONFIG.CurrentLimits.StatorCurrentLimit = 50;
        BALL_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        BALL_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimit = 50;
        
        // Pipe Intake Config
        PIPE_INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        PIPE_INTAKE_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        PIPE_INTAKE_CONFIG.CurrentLimits.StatorCurrentLimit = 50;
        PIPE_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        PIPE_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimit = 50;

        // Intake Pivot Config
        WRIST_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        WRIST_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 100;
        WRIST_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        WRIST_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        WRIST_CONFIG.CurrentLimits.StatorCurrentLimit = 50;
        WRIST_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        WRIST_CONFIG.CurrentLimits.SupplyCurrentLimit = 100;
    }

    /**
     * TODO:
     * - current limits
     */
}