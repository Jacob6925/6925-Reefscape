// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.CommandX3DController;
import frc.lib.TunerConstants;
import frc.robot.subsystems.BallIntakeSubsys;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.WristSubsys;
import frc.lib.util.GetPositionSubsys;
import frc.robot.subsystems.BallIntakeSubsys.BallIntakeSpeed;
import frc.robot.subsystems.ElevatorSubsys.ElevatorPosition;
import frc.robot.subsystems.PipeIntakeSubsys.PipeIntakeSpeed;
import frc.robot.subsystems.PipeIntakeSubsys;

public class RobotContainer {
    private static RobotContainer instance;
    private final ElevatorSubsys elevatorSubsys = new ElevatorSubsys();
    // private final PipeIntakeSubsys pipeIntakeSubsys = new PipeIntakeSubsys();
    // private final BallIntakeSubsys ballIntakeSubsys = new BallIntakeSubsys();
    // private final WristSubsys wristSubsys = new WristSubsys();

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandX3DController operator = new CommandX3DController(1);

    private final SendableChooser<Command> autoChooser;

    // GetPositionSubsys elevatorTesting = new GetPositionSubsys(
    //     "Elevator",
    //     12,
    //     13,
    //     false,
    //     (subsys) -> {
    //         subsys.mainMotor.getConfigurator().apply(Constants.Configs.ELEVATOR_CONFIG);
    //         subsys.subMotor.getConfigurator().apply(Constants.Configs.ELEVATOR_CONFIG);

    //         ElevatorFeedforward ff = new ElevatorFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG, Constants.ElevatorConstants.kV);
            
    //         subsys.maxes.reset();
    //         subsys.setDefaultCommand(Commands.runOnce(() -> {
    //             double input = MathUtil.applyDeadband(-driver.getLeftY(), 0.1);
    //             if (input != 0) {
    //                 double maxVelocity = 0.5;
    //                 subsys.mainMotor.set(MathUtil.clamp(input, -maxVelocity, maxVelocity));
    //             } else {
    //                 subsys.mainMotor.setVoltage(ff.calculate(0));
    //             }
    //             // subsys.mainMotor.set(MathUtil.applyDeadband(-driver.getLeftY(), 0.1));

    //             subsys.maxes.checkMaxVel(-subsys.mainMotor.getVelocity().getValueAsDouble());
    //             subsys.maxes.checkMaxAccel(-subsys.mainMotor.getAcceleration().getValueAsDouble());
    //             SmartDashboard.putNumber("EMaxVel", subsys.maxes.getMaxVel());
    //             SmartDashboard.putNumber("EMaxAccel", subsys.maxes.getMaxAccel());

    //             SmartDashboard.putNumber("Elevator motor Voltage", subsys.mainMotor.getMotorVoltage().getValueAsDouble());
    //         }, subsys));
    //     }
    // );

    // GetPositionSubsys wristTesting = new GetPositionSubsys(
    //     "Wrist",
    //     8,
    //     9,
    //     true,
    //     (subsys) -> {
    //         subsys.mainMotor.getConfigurator().apply(Constants.Configs.WRIST_CONFIG);
    //         subsys.setDefaultCommand(Commands.runOnce(() -> {
    //             SmartDashboard.putNumber("WristPos", subsys.mainMotor.getPosition().getValueAsDouble());
    //             SmartDashboard.putNumber("WristPos2", subsys.subMotor.getPosition().getValueAsDouble());
    //             subsys.mainMotor.set(MathUtil.applyDeadband(-driver.getRightY(), 0.1));
    //         }, subsys));
    //     }
    // );

    // GetPositionSubsys ballIntakeTesting = new GetPositionSubsys(
    //     "BallIntake",
    //     10,
    //     (subsys, motor) -> {
    //         motor.getConfigurator().apply(Constants.Configs.BALL_INTAKE_CONFIG);
    //         subsys.setDefaultCommand(Commands.runOnce(() -> {
    //             motor.set(MathUtil.applyDeadband(driver.getLeftX(), 0.1));
    //         }, subsys));
    //     }
    // );

    // GetPositionSubsys pipeIntakeTesting = new GetPositionSubsys(
    //     "PipeIntake",
    //     11,
    //     10,
    //     false,
    //     (subsys) -> {
    //         subsys.mainMotor.getConfigurator().apply(Constants.Configs.PIPE_INTAKE_CONFIG);
            
    //         subsys.setDefaultCommand(Commands.runOnce(() -> {
    //             subsys.mainMotor.set(MathUtil.applyDeadband(driver.getRightY(), 0.1));
    //         }, subsys));
    //     }
    // );


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

        CameraServer.startAutomaticCapture();
    }

    private void registerNamedCommands() {
        // NamedCommands.registerCommand("IntakePipe", new InstantCommand(() -> {
        //     pipeIntakeSubsys.setSpeed(PipeIntakeSpeed.INTAKE);
        //     ballIntakeSubsys.setSpeedFromPipeSpeed(PipeIntakeSpeed.INTAKE);
        // }, pipeIntakeSubsys));
        // NamedCommands.registerCommand("EjectPipe", new InstantCommand(() -> pipeIntakeSubsys.setSpeed(PipeIntakeSpeed.EJECT), pipeIntakeSubsys));
        // NamedCommands.registerCommand("IntakeBall", new InstantCommand(() -> ballIntakeSubsys.setSpeed(BallIntakeSpeed.INTAKE), ballIntakeSubsys));
        // NamedCommands.registerCommand("EjectBall", new InstantCommand(() -> ballIntakeSubsys.setSpeed(BallIntakeSpeed.EJECT), ballIntakeSubsys));
    }

    private void configureBindings() {
        // configureSwerveButtons();

        // // driver.b().onTrue(Commands.runOnce(() -> wristSubsys.wristMotor.setPosition(0), wristSubsys));
        // // driver.y().onTrue(Commands.runOnce(() -> wristSubsys.wristMotor.setPosition(90), wristSubsys));
        // // driver.x().onTrue(Commands.runOnce(() -> wristSubsys.wristMotor.setPosition(180), wristSubsys));
        // // driver.a().onTrue(Commands.runOnce(() -> wristSubsys.wristMotor.setPosition(270), wristSubsys));
        // // wristSubsys.setDefaultCommand(Commands.runOnce(() -> wristSubsys.wristMotor.set(driver.getLeftX()), wristSubsys));

        // driver.y().onTrue(elevatorSubsys.goTo(ElevatorPosition.MAX_HEIGHT));
        // driver.a().onTrue(elevatorSubsys.goTo(ElevatorPosition.QUARTER_HEIGHT));
    }

    private void configureSwerveButtons() {        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed * drivetrain.getCurrentSpeedMulti()) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed * drivetrain.getCurrentSpeedMulti()) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        driver.leftBumper().onChange(new InstantCommand(() -> drivetrain.toggleHalfSpeed()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static RobotContainer getInstance() {
        return instance;
    }
}
