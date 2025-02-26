package frc.lib.util;

/** Util class used for getting the max velocity and acceleration of a subsystem */
public class MaxFinder {
    private double maxVel = 0;
    private double maxAccel = 0;

    public void checkMaxVel(double input) {
        if (input > maxVel) {
            maxVel = input;
        }
    }

    public void checkMaxAccel(double input) {
        if (input > maxAccel) {
            maxAccel = input;
        }
    }

    public double getMaxVel() {
        return maxVel;
    }

    public double getMaxAccel() {
        return maxAccel;
    }

    public void reset() {
        maxAccel = 0;
        maxVel = 0;
    }
}
