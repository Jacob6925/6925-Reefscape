// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.CommandX3DController;
import frc.lib.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.IntakePivotSubsys;
import frc.robot.subsystems.IntakeSubsys;

public class RobotContainer {
    private static RobotContainer instance;
    private final ElevatorSubsys elevatorSubsys = new ElevatorSubsys();
    private final IntakeSubsys intakeSubsys = new IntakeSubsys();
    private final IntakePivotSubsys pivotSubsys = new IntakePivotSubsys();

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandX3DController operator = new CommandX3DController(1);

    private final SendableChooser<Command> autoChooser;



    /* ----------------- Generated Stuff ----------------- */
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    
    /* --------------------------------------------------- */

    public RobotContainer() {    
        instance = this;
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Selected Auto", autoChooser);

        registerNamedCommands();
        configureBindings();
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("", Commands.print("No command configured"));
    }

    private void configureBindings() {
        // // Note that X is defined as forward according to WPILib convention,
        // // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driver.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        // ));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        // driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // drivetrain.registerTelemetry(logger::telemeterize);

        driver.b().onTrue(Commands.runOnce(() -> pivotSubsys.pivotMotor.setPosition(0), pivotSubsys));
        driver.y().onTrue(Commands.runOnce(() -> pivotSubsys.pivotMotor.setPosition(90), pivotSubsys));
        driver.x().onTrue(Commands.runOnce(() -> pivotSubsys.pivotMotor.setPosition(180), pivotSubsys));
        driver.a().onTrue(Commands.runOnce(() -> pivotSubsys.pivotMotor.setPosition(270), pivotSubsys));

        pivotSubsys.setDefaultCommand(Commands.runOnce(() -> pivotSubsys.pivotMotor.set(driver.getLeftX()), pivotSubsys));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static RobotContainer getInstance() {
        return instance;
    }
}
