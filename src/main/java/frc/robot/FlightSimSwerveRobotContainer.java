// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.CommandX3DController;
import frc.lib.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FlightSimSwerveRobotContainer {
    private static FlightSimSwerveRobotContainer instance;

    private final CommandX3DController driver = new CommandX3DController(1);

    /* ----------------- Generated Stuff ----------------- */
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    /* --------------------------------------------------- */

    public FlightSimSwerveRobotContainer() {    
        instance = this;
        configureSwerveButtons();

        CameraServer.startAutomaticCapture(0).setConfigJson("{\"height\":120,\"pixelformat\":\"mjpeg\",\"properties\":[{\"name\":\"connect_verbose\",\"value\":1},{\"name\":\"raw_brightness\",\"value\":153},{\"name\":\"brightness\",\"value\":55},{\"name\":\"raw_contrast\",\"value\":2},{\"name\":\"contrast\",\"value\":20},{\"name\":\"raw_saturation\",\"value\":180},{\"name\":\"saturation\",\"value\":90},{\"name\":\"white_balance_temperature_auto\",\"value\":true},{\"name\":\"power_line_frequency\",\"value\":2},{\"name\":\"white_balance_temperature\",\"value\":4500},{\"name\":\"raw_sharpness\",\"value\":25},{\"name\":\"sharpness\",\"value\":50},{\"name\":\"backlight_compensation\",\"value\":0},{\"name\":\"exposure_auto\",\"value\":3},{\"name\":\"raw_exposure_absolute\",\"value\":156},{\"name\":\"exposure_absolute\",\"value\":46},{\"name\":\"pan_absolute\",\"value\":0},{\"name\":\"tilt_absolute\",\"value\":0},{\"name\":\"zoom_absolute\",\"value\":0}],\"width\":160}");
        // CameraServer.startAutomaticCapture(1);
        // http://roborio-6925-frc.local:1181/
    }

    private void configureSwerveButtons() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                drivetrain.setSpeedMulti(0.5*driver.getSlider() + 0.5);

                double velocityX = -driver.getPitch() * MaxSpeed * drivetrain.getCurrentSpeedMulti(); // Drive forward with negative Y (forward)
                double velocityY = -driver.getRoll() * MaxSpeed * drivetrain.getCurrentSpeedMulti(); // Drive left with negative X (left)
                double rotationalRate = -driver.getYaw() * MaxAngularRate; // Drive counterclockwise with negative X (left)

                return drive.withVelocityX(velocityX)
                    .withVelocityY(velocityY)
                    .withRotationalRate(rotationalRate);
            })
        );

        // reset the field-centric heading on left bumper press
        driver.button(11).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public static FlightSimSwerveRobotContainer getInstance() {
        return instance;
    }
}
