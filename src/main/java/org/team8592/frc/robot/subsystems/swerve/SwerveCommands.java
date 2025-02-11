package org.team8592.frc.robot.subsystems.swerve;

import org.team8592.frc.robot.subsystems.SubsystemCommands;
import org.team8592.frc.robot.subsystems.swerve.SwerveSubsystem.DriveModes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class SwerveCommands extends SubsystemCommands<SwerveSubsystem> {
    public SwerveCommands(SwerveSubsystem swerve) {
        super(swerve);
    }

    public Command joystickDriveCommand(double translateX, double translateY, double rotate) {
        return subsystem.run(() -> {
            subsystem.drive(subsystem.processJoystickInputs(
                translateX,
                translateY,
                rotate
            ), DriveModes.AUTOMATIC);
        }).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
}