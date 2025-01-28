package frc.lib;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandX3DController extends CommandGenericHID{
    public CommandX3DController(int port) {
        super(port);
    }

    public Trigger trigger() {
        return button(1);
    }

    public Trigger two() {
        return button(2);
    }

    public Trigger three() {
        return button(3);
    }

    public Trigger four() {
        return button(4);
    }

    public Trigger five() {
        return button(5);
    }

    public Trigger six() {
        return button(6);
    }

    public Trigger seven() {
        return button(7);
    }

    public Trigger eight() {
        return button(8);
    }

    public Trigger nine() {
        return button(9);
    }

    public Trigger ten() {
        return button(10);
    }

    public Trigger eleven() {
        return button(11);
    }

    public Trigger twelve() {
        return button(12);
    }

    public double getPitch() {
        return getHID().getRawAxis(1);
    }

    public double getRoll() {
        return getHID().getRawAxis(0);
    }

    public double getYaw() {
        return getHID().getRawAxis(2);
    }

    public double getSlider() {
        return getHID().getRawAxis(3);
    }
}
