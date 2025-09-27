package frc.robot.io;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Button {
    private final GenericHID hidDevice;
    private final int id;
    private int iteration = 0;

    Button(GenericHID hidDevice, int id) {
        this.hidDevice = hidDevice;
        this.id = id;
    }

    public Trigger trigger() {
        return new Trigger(() -> hidDevice.getRawButtonPressed(id));
    }

    public int getId() {
        return id;
    }

    public void on() {
        hidDevice.setOutput(id, true);
        iteration = 0;
    }

    public void off() {
        hidDevice.setOutput(id, false);
        iteration = 0;
    }

    public void blink() {
        hidDevice.setOutput(id, (iteration / 60) % 2 == 0);
        iteration++;
    }
}
