// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SimulationRobotContainer {
    private static SimulationRobotContainer instance;

    private final CommandXboxController driver = new CommandXboxController(0);

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
    
    private final SwerveRequest.FieldCentricFacingAngle driveToAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(0)
        // .withDeadband(MAX_SPEED * 0.035)
        .withDriveRequestType(DriveRequestType.Velocity);
        
    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Field2d field;
    
    /* --------------------------------------------------- */

    public SimulationRobotContainer() {    
        instance = this;
        configureSwerveButtons();

        drivetrain.setupPhotonVision();
        driveToAngle.HeadingController = new PhoenixPIDController(10, 0.0, 0.0);
        driveToAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    private void configureSwerveButtons() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                SmartDashboard.putBoolean("Field Oriented", fieldCentric);

                int driverPov = driver.getHID().getPOV();
                if (driverPov != -1) {
                    double x = 0;
                    double y = 0;
                    double MOVE_VEL = 0.5;
                    switch (driverPov) {
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

                velocityX *= drivetrain.getCurrentSpeedMulti();
                velocityY *= drivetrain.getCurrentSpeedMulti();

                if (drivetrain.autoAligning()) {
                    return driveToAngle
                        .withTargetDirection(new Rotation2d(180));
                } else if (fieldCentric) {
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

        // reset the field-centric heading on left bumper press
        driver.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        driver.leftBumper().onTrue(drivetrain.toggleSpeedMulti(1/3.0));
        driver.rightBumper().onTrue(new InstantCommand(() -> fieldCentric = !fieldCentric));
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public static SimulationRobotContainer getInstance() {
        return instance;
    }
}
