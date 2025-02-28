package frc.lib.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/** Util class used for getting the max velocity and acceleration of a subsystem */
public class MaxCollections {
    private final String id;
    private final DoubleSupplier speedSupplier;
    private final DoubleSupplier accelerationSupplier;

    private boolean collectData = false;
    private boolean periodicInitiated = false;
    private double maxSpeed = 0;
    private double maxAccel = 0;

    public MaxCollections(String id, DoubleSupplier speedSupplier, DoubleSupplier accelerationSupplier) {
        this.id = id;
        this.speedSupplier = speedSupplier;
        this.accelerationSupplier = accelerationSupplier;

        SmartDashboard.putNumber("Max Speed (" + id + ")", 0);
        SmartDashboard.putNumber("Max Acceleration (" + id + ")", 0);
    }

    public void startCollections() {
        collectData = true;
        if (!periodicInitiated) {
            periodicInitiated = true;
            Robot.getInstance().addPeriodic(() -> {
                if (collectData) {
                    if (speedSupplier != null) {
                        double currSpeed = speedSupplier.getAsDouble();
                        if (currSpeed > maxSpeed) {
                            maxSpeed = currSpeed;
                            SmartDashboard.putNumber("Max Speed (" + id + ")", maxSpeed);
                        }
                    }
                    
                    if (accelerationSupplier != null) {
                        double currAccel = accelerationSupplier.getAsDouble();
                        if (currAccel > maxAccel) {
                            maxAccel = currAccel;
                            SmartDashboard.putNumber("Max Acceleration (" + id + ")", maxAccel);
                        }
                    }
                }
            }, 1);
        }
    }

    public void stopCollection() {
        collectData = false;
    }

    public void reset() {
        maxSpeed = 0;
        maxAccel = 0;
    }
}
