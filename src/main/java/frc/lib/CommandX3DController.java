package frc.lib;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandX3DController extends CommandGenericHID {
    public CommandX3DController(int port) {
        super(port);
    }

    public Trigger trigger() {
        return button(1);
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
