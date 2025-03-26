package frc.robot;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

public class InputStreams {
    final SwerveInputStream fieldOriented;
    final SwerveInputStream robotOriented;
    final SwerveInputStream fieldOrientedSlow;
    final SwerveInputStream robotOrientedSlow;

    public InputStreams(SwerveDrive swerve, ControlInputs inputs) {
        var base = SwerveInputStream.of(swerve,
            () -> {
                var power = -inputs.getDrivePower().y();
                return power * Math.abs(power);
            },
            () -> {
                var power = inputs.getDrivePower().x();
                return power * Math.abs(power);
            })
            .withControllerRotationAxis(() -> {
                var power = -inputs.getDrivePower().rotation();
                return power * Math.abs(power);
            })
            .deadband(Constants.DRIVE_STICK_DEADBAND);
        fieldOriented = base.copy().allianceRelativeControl(true)
            .scaleTranslation(0.8)
            .scaleRotation(0.5);
        robotOriented = base.copy().robotRelative(true)
            .scaleTranslation(0.8)
            .scaleRotation(0.5);
        fieldOrientedSlow = base.copy().allianceRelativeControl(true)
            .scaleTranslation(0.4)
            .scaleRotation(0.25);
        robotOrientedSlow = base.copy().robotRelative(true)
            .scaleTranslation(0.4)
            .scaleRotation(0.25);
    }
}
