// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.CommandX3DController;
import frc.lib.TunerConstants;
import frc.robot.subsystems.BallIntakeSubsys;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.WristSubsys;
import frc.robot.subsystems.BallIntakeSubsys.BallIntakeSpeed;
import frc.robot.subsystems.ElevatorSubsys.ElevatorPosition;
import frc.robot.subsystems.PipeIntakeSubsys.PipeIntakeSpeed;
import frc.robot.subsystems.WristSubsys.WristSetpoint;
import frc.robot.subsystems.PipeIntakeSubsys;

public class RobotContainer {
    private static RobotContainer instance;

    private final ElevatorSubsys elevatorSubsys;
    private final PipeIntakeSubsys pipeIntakeSubsys;
    private final BallIntakeSubsys ballIntakeSubsys;
    private final WristSubsys wristSubsys;

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

        elevatorSubsys = new ElevatorSubsys();
        pipeIntakeSubsys = new PipeIntakeSubsys();
        ballIntakeSubsys = new BallIntakeSubsys();
        wristSubsys = new WristSubsys();

        // elevatorSubsys = null;
        // pipeIntakeSubsys = null;
        // ballIntakeSubsys = null;
        // wristSubsys = null;

        // GetPositionSubsys elevatorTesting = new GetPositionSubsys(
        //     "Elevator",
        //     12,
        //     13,
        //     false,
        //     (subsys) -> {
        //         subsys.mainMotor.getConfigurator().apply(Constants.Configs.ELEVATOR_CONFIG);
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
        //             SmartDashboard.putNumber("Position", subsys.mainMotor.getPosition().getValueAsDouble());
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
        //             subsys.mainMotor.set(MathUtil.applyDeadband(-operator.getPitch(), 0.1));
        //             // SmartDashboard.putNumber("Wrist Voltage", subsys.mainMotor.getMotorVoltage().getValueAsDouble());
        //         }, subsys));
        //     }
        // );

        // GetPositionSubsys ballIntakeTesting = new GetPositionSubsys(
        //     "BallIntake",
        //     10,
        //     (subsys) -> {
        //         subsys.mainMotor.getConfigurator().apply(Constants.Configs.BALL_INTAKE_CONFIG);
        //         subsys.setDefaultCommand(Commands.runOnce(() -> {
        //             subsys.mainMotor.set(MathUtil.applyDeadband(-driver.getLeftX(), 0.1));
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
        //             subsys.mainMotor.set(MathUtil.applyDeadband(driver.getRightX(), 0.1));
        //         }, subsys));
        //     }
        // );

        registerNamedCommands();
        configureBindings();

        CameraServer.startAutomaticCapture(0);
        CameraServer.startAutomaticCapture(1);
        // http://roborio-6925-frc.local:1181/
    }

    private void configureBindings() {
        configureSwerveButtons();

        if (elevatorSubsys != null) {
            if (pipeIntakeSubsys != null && ballIntakeSubsys != null) {
                operator.button(1).onTrue(Commands.runOnce(() -> {
                    PipeIntakeSpeed speed;
                    if (Math.abs(elevatorSubsys.getPIDController().getGoal().position - elevatorSubsys.getMotorRotations()) < 1) {
                        speed = PipeIntakeSpeed.L1_EJECT;
                    } else {
                        speed = PipeIntakeSpeed.EJECT;
                    }
                    setSpeedPipeAndBall(speed).initialize();
                }, pipeIntakeSubsys, ballIntakeSubsys)).onFalse(setSpeedPipeAndBall(PipeIntakeSpeed.OFF));

                operator.button(2).onTrue(humanPlayerActionsCommand).onFalse(
                    Commands.sequence(
                        pipeIntakeSubsys.setSpeedCommand(PipeIntakeSpeed.OFF),
                        ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.OFF),
                        resetElevator()
                    )
                );
            }

            operator.button(10).onTrue(elevatorSubsys.goTo(ElevatorPosition.L4)).onFalse(resetElevator());
            operator.button(9).onTrue(elevatorSubsys.goTo(ElevatorPosition.L3)).onFalse(resetElevator());
            operator.button(12).onTrue(elevatorSubsys.goTo(ElevatorPosition.L2)).onFalse(resetElevator());
            operator.button(11).onTrue(elevatorSubsys.goTo(ElevatorPosition.L1)).onFalse(resetElevator());
        }
    }

    private Command setSpeedPipeAndBall(PipeIntakeSpeed speed) {
        return Commands.runOnce(() -> {
            pipeIntakeSubsys.setSpeed(speed);
            ballIntakeSubsys.setSpeedFromPipeSpeed(speed);
        }, pipeIntakeSubsys, ballIntakeSubsys);
    }

    private Command resetElevator() {
        return Commands.sequence(
            wristSubsys.goTo(WristSetpoint.START_POS),
            elevatorSubsys.goTo(ElevatorPosition.MIN_HEIGHT)
        );
    }

    private void configureSwerveButtons() {
        final double REDUCE_SPEED_WHEN_ELEV_UP = 7.5;//5.0;

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double velocityX = -driver.getLeftY() * MaxSpeed * drivetrain.getCurrentSpeedMulti(); // Drive forward with negative Y (forward)
                double velocityY = -driver.getLeftX() * MaxSpeed * drivetrain.getCurrentSpeedMulti(); // Drive left with negative X (left)
                double rotationalRate = -driver.getRightX() * MaxAngularRate; // Drive counterclockwise with negative X (left)

                if (elevatorSubsys != null && elevatorSubsys.getMotorRotations() > 1) {
                    velocityX /= REDUCE_SPEED_WHEN_ELEV_UP;
                    velocityY /= REDUCE_SPEED_WHEN_ELEV_UP;
                }

                return drive.withVelocityX(velocityX)
                    .withVelocityY(velocityY)
                    .withRotationalRate(rotationalRate);
            })
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

        driver.leftBumper().onTrue(drivetrain.toggleHalfSpeed());
    }

    private Command humanPlayerActionsCommand;

    private void registerNamedCommands() {
        humanPlayerActionsCommand = new SequentialCommandGroup(
                elevatorSubsys.goTo(ElevatorPosition.HUMAN_PLAYER_INTAKE),
                new WaitCommand(0.5),
                wristSubsys.goTo(WristSetpoint.HUMAN_PLAYER_INTAKE)
            ).andThen(
                pipeIntakeSubsys.setSpeedCommand(PipeIntakeSpeed.INTAKE),
                ballIntakeSubsys.setSpeedFromPipeSpeedCommand(PipeIntakeSpeed.INTAKE)
        );

        NamedCommands.registerCommand("MoveTo-L1", elevatorSubsys.goTo(ElevatorPosition.L1));
        NamedCommands.registerCommand("MoveTo-L2", elevatorSubsys.goTo(ElevatorPosition.L2));
        NamedCommands.registerCommand("MoveTo-L3", elevatorSubsys.goTo(ElevatorPosition.L3));
        NamedCommands.registerCommand("MoveTo-L4", elevatorSubsys.goTo(ElevatorPosition.L4));
        NamedCommands.registerCommand("MoveTo-HP", humanPlayerActionsCommand);
        NamedCommands.registerCommand("ResetElevator", new InstantCommand(() -> elevatorSubsys.resetElevatorSetpoint(), elevatorSubsys));
        NamedCommands.registerCommand("IntakePipe", new SequentialCommandGroup(
            pipeIntakeSubsys.setSpeedCommand(PipeIntakeSpeed.INTAKE),
            ballIntakeSubsys.setSpeedFromPipeSpeedCommand(PipeIntakeSpeed.INTAKE)
        ));
        NamedCommands.registerCommand("EjectPipe", new SequentialCommandGroup(
            pipeIntakeSubsys.setSpeedCommand(PipeIntakeSpeed.EJECT),
            ballIntakeSubsys.setSpeedFromPipeSpeedCommand(PipeIntakeSpeed.EJECT)
        ));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static RobotContainer getInstance() {
        return instance;
    }
}
