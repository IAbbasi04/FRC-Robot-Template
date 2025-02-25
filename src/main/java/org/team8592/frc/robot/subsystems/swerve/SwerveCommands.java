package org.team8592.frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import org.team8592.frc.robot.subsystems.SubsystemCommands;
import org.team8592.frc.robot.subsystems.swerve.SwerveSubsystem.DriveModes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class SwerveCommands extends SubsystemCommands<SwerveSubsystem> {
    public SwerveCommands(SwerveSubsystem swerve) {
        super(swerve);
    }

    /**
     * Command to drive the swerve with translation and rotation processed for human input
     * 
     * @param translateX a lambda returning the driver's X input
     * @param translateY a lambda returning the driver's Y input
     * @param rotate a lambda returning the driver's rotate input
     *
     * @return the command
     */
    public Command joystickDriveCommand(DoubleSupplier translateX, DoubleSupplier translateY, DoubleSupplier rotate) {
        return subsystem.run(() -> {
            subsystem.drive(subsystem.processJoystickInputs(
                translateX.getAsDouble(),
                translateY.getAsDouble(),
                rotate.getAsDouble()
            ), DriveModes.AUTOMATIC);
        }).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    /**
     * Command to drive the swerve with translation processed for human input and
     * rotation controlled by the snap-to PID controller (snapping to the passed-in)
     * angle
     *
     * @param angle the angle to snap to
     * @param translateX a lambda returning the driver's X input
     * @param translateY a lambda returning the driver's Y input
     *
     * @return the command
     */
    public Command snapToAngleCommand(Rotation2d angle, DoubleSupplier translateX, DoubleSupplier translateY) {
        return subsystem.run(() -> {
            subsystem.drive(
                subsystem.processJoystickInputs(
                translateX.getAsDouble(),
                translateY.getAsDouble(),
                subsystem.snapToAngle(angle)
            ), DriveModes.AUTOMATIC);
        });
    }
}