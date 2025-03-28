// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.CommandX3DController;
import frc.lib.TunerConstants;
import frc.robot.subsystems.BallIntakeSubsys;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ElevatorSubsys.ElevatorPosition;
import frc.robot.subsystems.PipeIntakeSubsys;
import frc.robot.subsystems.PipeIntakeSubsys.PipeIntakeSpeed;
import frc.robot.subsystems.WristSubsys;
import frc.robot.subsystems.BallIntakeSubsys.BallIntakeSpeed;
import frc.robot.subsystems.WristSubsys.WristSetpoint;

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
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private boolean fieldCentric = true;
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    /* --------------------------------------------------- */

    public RobotContainer() {    
        instance = this;

        elevatorSubsys = new ElevatorSubsys();
        pipeIntakeSubsys = new PipeIntakeSubsys();
        ballIntakeSubsys = new BallIntakeSubsys();
        wristSubsys = new WristSubsys();

        RobotCommands.init(elevatorSubsys, pipeIntakeSubsys, ballIntakeSubsys, wristSubsys);

        registerNamedCommands();
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Selected Auto", autoChooser);

        CameraServer.startAutomaticCapture(0).setConfigJson("{\"height\":120,\"pixelformat\":\"mjpeg\",\"properties\":[{\"name\":\"connect_verbose\",\"value\":1},{\"name\":\"raw_brightness\",\"value\":153},{\"name\":\"brightness\",\"value\":55},{\"name\":\"raw_contrast\",\"value\":2},{\"name\":\"contrast\",\"value\":20},{\"name\":\"raw_saturation\",\"value\":180},{\"name\":\"saturation\",\"value\":90},{\"name\":\"white_balance_temperature_auto\",\"value\":true},{\"name\":\"power_line_frequency\",\"value\":2},{\"name\":\"white_balance_temperature\",\"value\":4500},{\"name\":\"raw_sharpness\",\"value\":25},{\"name\":\"sharpness\",\"value\":50},{\"name\":\"backlight_compensation\",\"value\":0},{\"name\":\"exposure_auto\",\"value\":3},{\"name\":\"raw_exposure_absolute\",\"value\":156},{\"name\":\"exposure_absolute\",\"value\":46},{\"name\":\"pan_absolute\",\"value\":0},{\"name\":\"tilt_absolute\",\"value\":0},{\"name\":\"zoom_absolute\",\"value\":0}],\"width\":160}");
        // CameraServer.startAutomaticCapture(1);
        // http://roborio-6925-frc.local:1181/
    }

    private void configureBindings() {
        configureSwerveButtons();
        operator.trigger().onTrue(new ConditionalCommand(
            RobotCommands.setSpeedPipeAndBall(PipeIntakeSpeed.L1_EJECT), //true
            RobotCommands.setSpeedPipeAndBall(PipeIntakeSpeed.EJECT), //false
            () -> elevatorSubsys.getPIDController().getGoal().position == ElevatorPosition.L1.rotations) //condition - at L1
        ).onFalse(RobotCommands.setSpeedPipeAndBall(PipeIntakeSpeed.OFF));

        operator.button(2).onTrue(RobotCommands.humanPlayerActionsCommand()).onFalse(RobotCommands.resetIntakesAndElevator());

        operator.button(10).onTrue(RobotCommands.elevatorLevelActions(ElevatorPosition.L4, WristSetpoint.L4)).onFalse(RobotCommands.resetElevator());
        operator.button(9).onTrue(RobotCommands.elevatorLevelActions(ElevatorPosition.L3, WristSetpoint.L2_L3)).onFalse(RobotCommands.resetElevator());
        operator.button(12).onTrue(RobotCommands.elevatorLevelActions(ElevatorPosition.L2, WristSetpoint.L2_L3)).onFalse(RobotCommands.resetElevator());
        operator.button(11).onTrue(elevatorSubsys.goTo(ElevatorPosition.L1)).onFalse(RobotCommands.resetElevator());

        operator.button(7).onTrue(RobotCommands.moveElevAndIntakeBall(ElevatorPosition.REMOVE_ALGAE_L2)).onFalse(RobotCommands.resetIntakesAndElevator());
        operator.button(8).onTrue(RobotCommands.moveElevAndIntakeBall(ElevatorPosition.REMOVE_ALGAE_L3)).onFalse(RobotCommands.resetIntakesAndElevator());
        // operator.button(3).onTrue(RobotCommands.liftAndShoot());
        operator.button(6).onTrue(RobotCommands.outtakeBall(ElevatorPosition.DEPOSIT_BALL)).onFalse(RobotCommands.resetIntakesAndElevator());
        operator.button(4).onTrue(ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.HOLD_BALL)).onFalse(ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.OFF));

        driver.button(8).onTrue(elevatorSubsys.setSpeedCommand(-0.025)).onFalse(new InstantCommand(() -> elevatorSubsys.resetElevatorSetpoint()));
    }

    private void configureSwerveButtons() {
        final double REDUCE_SPEED_WHEN_ELEV_UP = 6.0;

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                SmartDashboard.putBoolean("Field Oriented", fieldCentric);

                int pov = driver.getHID().getPOV();
                if (pov != -1) {
                    double x = 0;
                    double y = 0;
                    double MOVE_VEL = 0.5;
                    switch (pov) {
                        case 0 -> x = MOVE_VEL;
                        case 90 -> y = -MOVE_VEL;
                        case 180 -> x = -MOVE_VEL;
                        case 270 -> y = MOVE_VEL;
                    }
                    if (x != 0 || y != 0) return robotCentric.withVelocityX(x).withVelocityY(y);
                }

                double velocityX = -driver.getLeftY() * MaxSpeed; // Drive forward with negative Y (forward)
                double velocityY = -driver.getLeftX() * MaxSpeed; // Drive left with negative X (left)
                double rotationalRate = -driver.getRightX() * MaxAngularRate; // Drive counterclockwise with negative X (left)

                if (elevatorSubsys.getMotorRotations() > 1) {
                    velocityX /= REDUCE_SPEED_WHEN_ELEV_UP;
                    velocityY /= REDUCE_SPEED_WHEN_ELEV_UP;
                } else {
                    velocityX *= drivetrain.getCurrentSpeedMulti();
                    velocityY *= drivetrain.getCurrentSpeedMulti();
                } 

                if (fieldCentric) {
                    return drive.withVelocityX(velocityX)
                        .withVelocityY(velocityY)
                        .withRotationalRate(rotationalRate);
                } else {
                    return robotCentric.withVelocityX(velocityX)
                        .withVelocityY(velocityY)
                        .withRotationalRate(rotationalRate);
                }
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

        driver.leftBumper().onTrue(drivetrain.toggleSpeedMulti(1/3.0));
        driver.rightBumper().onTrue(new InstantCommand(() -> fieldCentric = !fieldCentric));
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("MoveTo-L1", elevatorSubsys.goTo(ElevatorPosition.L1));
        NamedCommands.registerCommand("MoveTo-L2", RobotCommands.elevatorLevelActions(ElevatorPosition.L2, WristSetpoint.L2_L3));
        NamedCommands.registerCommand("MoveTo-L3", RobotCommands.elevatorLevelActions(ElevatorPosition.L3, WristSetpoint.L2_L3));
        NamedCommands.registerCommand("MoveTo-L4", RobotCommands.elevatorLevelActions(ElevatorPosition.L4, WristSetpoint.L4));
        NamedCommands.registerCommand("MoveTo-HP", RobotCommands.humanPlayerActionsCommand());
        NamedCommands.registerCommand("ResetElevator", RobotCommands.resetElevator());
        NamedCommands.registerCommand("IntakePipe", RobotCommands.setSpeedPipeAndBall(PipeIntakeSpeed.INTAKE));
        NamedCommands.registerCommand("EjectPipe", RobotCommands.ejectIntakes(true, true));
        NamedCommands.registerCommand("EjectBall", RobotCommands.ejectIntakes(false, true));
        NamedCommands.registerCommand("StopIntake", RobotCommands.setSpeedPipeAndBall(PipeIntakeSpeed.OFF));
        NamedCommands.registerCommand("DescoreL2", RobotCommands.moveElevAndIntakeBall(ElevatorPosition.REMOVE_ALGAE_L2));
        NamedCommands.registerCommand("DescoreL3", RobotCommands.moveElevAndIntakeBall(ElevatorPosition.REMOVE_ALGAE_L3));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static RobotContainer getInstance() {
        return instance;
    }
}
