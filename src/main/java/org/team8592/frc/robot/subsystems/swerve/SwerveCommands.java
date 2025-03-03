package org.team8592.frc.robot.subsystems.swerve;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.team8592.frc.robot.subsystems.swerve.SwerveSubsystem.DriveModes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;

public class SwerveCommands {
    private SwerveSubsystem swerve;
    public SwerveCommands(SwerveSubsystem swerve) {
        this.swerve = swerve;
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
        return new DeferredCommand(() -> swerve.run(() -> {
            swerve.drive(swerve.processJoystickInputs(
                translateX.getAsDouble(),
                translateY.getAsDouble(),
                rotate.getAsDouble()
            ), DriveModes.AUTOMATIC);
        }), Set.of(swerve));
    }

    public Command joystickDriveCommand(double translateX, double translateY, double rotate) {
        return swerve.run(() -> {
            SmartDashboard.putNumber("AAJKLJKLDSA", translateX);

            swerve.drive(swerve.processJoystickInputs(
                translateX, 
                translateY, 
                rotate
            ));
        });
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
        return swerve.run(() -> {
            swerve.drive(
                swerve.processJoystickInputs(
                translateX.getAsDouble(),
                translateY.getAsDouble(),
                swerve.snapToAngle(angle)
            ), DriveModes.AUTOMATIC);
        });
    }

    public Command followPathCommand(Trajectory trajectory, BooleanSupplier flipForRed) {
        return Commands.none(); // TODO - Add this later
    }
}