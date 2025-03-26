// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.CommandX3DController;
import frc.lib.util.GetPositionSubsys;

public class TestingRobotContainer {
    private static TestingRobotContainer instance;

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandX3DController operator = new CommandX3DController(1);
    
    /* --------------------------------------------------- */

    public TestingRobotContainer() {
        instance = this;

        /* Elevator */
        new GetPositionSubsys(
            "Elevator",
            12,
            13,
            false,
            (subsys) -> {
                subsys.mainMotor.getConfigurator().apply(Constants.Configs.ELEVATOR_CONFIG);
                ElevatorFeedforward ff = new ElevatorFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG, Constants.ElevatorConstants.kV);
                subsys.maxes.reset();
                subsys.setDefaultCommand(new InstantCommand(() -> {
                    double input = MathUtil.applyDeadband(-driver.getLeftY(), 0.1);
                    if (input != 0) {
                        double maxVelocity = 0.25;
                        subsys.mainMotor.set(MathUtil.clamp(input, -maxVelocity, maxVelocity));
                    } else {
                        subsys.mainMotor.setVoltage(ff.calculate(0));
                    }
                    // subsys.mainMotor.set(MathUtil.applyDeadband(-driver.getLeftY(), 0.1));
                }, subsys));
            }
        );

        /* Wrist */
        new GetPositionSubsys(
            "Wrist",
            8,
            9,
            true,
            (subsys) -> {
                subsys.mainMotor.getConfigurator().apply(Constants.Configs.WRIST_CONFIG);
                subsys.setDefaultCommand(new InstantCommand(() -> {
                    subsys.mainMotor.set(MathUtil.applyDeadband(-operator.getPitch(), 0.1));
                }, subsys));
            }
        );

        /* Ball Intake */
        new GetPositionSubsys(
            "BallIntake",
            10,
            (subsys) -> {
                subsys.mainMotor.getConfigurator().apply(Constants.Configs.BALL_INTAKE_CONFIG);
                subsys.setDefaultCommand(new InstantCommand(() -> {
                    subsys.mainMotor.set(MathUtil.applyDeadband(-driver.getLeftX(), 0.1));
                }, subsys));
            }
        );

        // /* Pipe Intake */
        // new GetPositionSubsys(
        //     "PipeIntake",
        //     11,
        //     10,
        //     false,
        //     (subsys) -> {
        //         subsys.mainMotor.getConfigurator().apply(Constants.Configs.PIPE_INTAKE_CONFIG);
        //         subsys.setDefaultCommand(new InstantCommand(() -> {
        //             subsys.mainMotor.set(MathUtil.applyDeadband(driver.getRightX(), 0.1));
        //         }, subsys));
        //     }
        // );
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public static TestingRobotContainer getInstance() {
        return instance;
    }
}
