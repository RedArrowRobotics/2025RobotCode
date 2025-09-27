package frc.robot.io;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RadioButtonGroup {
    private final GenericHID hidDevice;
    private final List<Button> buttons = new ArrayList<>();

    public RadioButtonGroup(GenericHID hidDevice) {
        this.hidDevice = hidDevice;
    }

    public Trigger addButton(int id) {
        return addButton(id, () -> true);
    }

    public Trigger addButton(int id, BooleanSupplier condition) {
        Button button = new Button(hidDevice, id);
        buttons.add(button);
        return button.trigger().onTrue(
                Commands.runEnd(
                        () -> {
                            buttons.forEach(b -> {
                                if (id == b.getId()) {
                                    b.blink();
                                } else {
                                    b.off();
                                }
                            });
                        },
                        () -> {
                            buttons.forEach(b -> {
                                if (id == b.getId()) {
                                    b.on();
                                } else {
                                    b.off();
                                }
                            });
                        }
                ).until(condition)
        );
    }
}
