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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    private final RobotCommands allCommands;

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
        //         subsys.setDefaultCommand(new InstantCommand(() -> {
        //             double input = MathUtil.applyDeadband(-driver.getLeftY(), 0.1);
        //             if (input != 0) {
        //                 double maxVelocity = 0.5;
        //                 subsys.mainMotor.set(MathUtil.clamp(input, -maxVelocity, maxVelocity));
        //             } else {
        //                 subsys.mainMotor.setVoltage(ff.calculate(0));
        //             }
        //             // subsys.mainMotor.set(MathUtil.applyDeadband(-driver.getLeftY(), 0.1));
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
        //         subsys.setDefaultCommand(new InstantCommand(() -> {
        //             subsys.mainMotor.set(MathUtil.applyDeadband(-operator.getPitch(), 0.1));
        //         }, subsys));
        //     }
        // );

        // GetPositionSubsys ballIntakeTesting = new GetPositionSubsys(
        //     "BallIntake",
        //     10,
        //     (subsys) -> {
        //         subsys.mainMotor.getConfigurator().apply(Constants.Configs.BALL_INTAKE_CONFIG);
        //         subsys.setDefaultCommand(new InstantCommand(() -> {
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
        //         subsys.setDefaultCommand(new InstantCommand(() -> {
        //             subsys.mainMotor.set(MathUtil.applyDeadband(driver.getRightX(), 0.1));
        //         }, subsys));
        //     }
        // );

        registerNamedCommands();
        configureBindings();

        CameraServer.startAutomaticCapture(0);
        CameraServer.startAutomaticCapture(1);
        // http://roborio-6925-frc.local:1181/

        allCommands = new RobotCommands(elevatorSubsys, pipeIntakeSubsys, ballIntakeSubsys, wristSubsys);
    }

    private void configureBindings() {
        configureSwerveButtons();

        operator.button(1).onTrue(new InstantCommand(() -> {
            PipeIntakeSpeed speed;
            if (Math.abs(elevatorSubsys.getPIDController().getGoal().position - elevatorSubsys.getMotorRotations()) < ElevatorPosition.L1.rotations+1) {
                speed = PipeIntakeSpeed.L1_EJECT;
            } else {
                speed = PipeIntakeSpeed.EJECT;
            }
            allCommands.setSpeedPipeAndBall(speed).initialize();
        }, pipeIntakeSubsys, ballIntakeSubsys)).onFalse(allCommands.setSpeedPipeAndBall(PipeIntakeSpeed.OFF));

        operator.button(2).onTrue(allCommands.humanPlayerActionsCommand()).onFalse(allCommands.resetIntakesAndElevator());

        operator.button(10).onTrue(allCommands.elevatorLevelActions(ElevatorPosition.L4, WristSetpoint.L4)).onFalse(allCommands.resetElevator());
        operator.button(9).onTrue(allCommands.elevatorLevelActions(ElevatorPosition.L3, WristSetpoint.L2_L3)).onFalse(allCommands.resetElevator());
        operator.button(12).onTrue(allCommands.elevatorLevelActions(ElevatorPosition.L2, WristSetpoint.L2_L3)).onFalse(allCommands.resetElevator());
        operator.button(11).onTrue(elevatorSubsys.goTo(ElevatorPosition.L1)).onFalse(allCommands.resetElevator());

        operator.button(7).onTrue(allCommands.moveElevAndIntakeBall(ElevatorPosition.REMOVE_ALGAE_L2)).onFalse(allCommands.resetIntakesAndElevator());
        operator.button(8).onTrue(allCommands.moveElevAndIntakeBall(ElevatorPosition.REMOVE_ALGAE_L3)).onFalse(allCommands.resetIntakesAndElevator());
        operator.button(-1).onTrue(ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.EJECT)).onFalse(ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.OFF));
    }

    private void configureSwerveButtons() {
        final double REDUCE_SPEED_WHEN_ELEV_UP = 5.0;

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
        driver.y().onTrue(new ParallelCommandGroup(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric()),
            wristSubsys.goTo(WristSetpoint.START_POS)
        ));

        drivetrain.registerTelemetry(logger::telemeterize);

        driver.leftBumper().onTrue(drivetrain.toggleSpeedMulti(1/3.0));
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("MoveTo-L1", elevatorSubsys.goTo(ElevatorPosition.L1));
        NamedCommands.registerCommand("MoveTo-L2", elevatorSubsys.goTo(ElevatorPosition.L2));
        NamedCommands.registerCommand("MoveTo-L3", elevatorSubsys.goTo(ElevatorPosition.L3));
        NamedCommands.registerCommand("MoveTo-L4", elevatorSubsys.goTo(ElevatorPosition.L4));
        NamedCommands.registerCommand("MoveTo-HP", allCommands.humanPlayerActionsCommand());
        NamedCommands.registerCommand("ResetElevator", allCommands.resetElevator());
        NamedCommands.registerCommand("IntakePipe", allCommands.setSpeedPipeAndBall(PipeIntakeSpeed.INTAKE));
        NamedCommands.registerCommand("EjectPipe", allCommands.setSpeedPipeAndBall(PipeIntakeSpeed.EJECT));
        NamedCommands.registerCommand("StopIntake", allCommands.setSpeedPipeAndBall(PipeIntakeSpeed.OFF));
        NamedCommands.registerCommand("DescoreL2", allCommands.moveElevAndIntakeBall(ElevatorPosition.REMOVE_ALGAE_L2));
        NamedCommands.registerCommand("DescoreL3", allCommands.moveElevAndIntakeBall(ElevatorPosition.REMOVE_ALGAE_L3));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static RobotContainer getInstance() {
        return instance;
    }
}
