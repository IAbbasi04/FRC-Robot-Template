// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team8592.frc.robot;

import static org.team8592.frc.robot.commands.NewtonCommands.*;

import org.team8592.frc.robot.commands.autonomous.*;
import org.team8592.frc.robot.commands.largecommands.LargeCommand;
import org.team8592.frc.robot.subsystems.SubsystemManager;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;


public class RobotContainer {
    // The robot's subsystems
    private final SubsystemManager manager;

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer(boolean logToShuffleboard) {
        manager = new SubsystemManager(logToShuffleboard);

        passSubsystems();
        configureDefaults();
        configureBindings();

        AutoManager.prepare();
    }

    /**
     * Pass subsystems everywhere they're needed
     */
    private void passSubsystems(){
        AutoManager.addSubsystems(manager);
        AutoCommand.addSubsystems(manager);
        LargeCommand.addSubsystems(manager);
    }

    /**
     * Configure default commands for the subsystems
     */
    private void configureDefaults(){
        // Set the swerve's default command to drive with joysticks
        manager.swerveSubsystem.setDefaultCommand(
            manager.swerveSubsystem.commands.joystickDriveCommand(
                Controls.driveTranslateX.getAsDouble(),
                Controls.driveTranslateY.getAsDouble(),
                Controls.driveRotate.getAsDouble()
            )
        );
    }


    //Any commands that are reused a lot but can't go in a separate class go here

    /**
     * Configure all button bindings
     */
    private void configureBindings() {
        // Driver controls:
        // Operator:
        Controls.slowMode.onTrue(
            // The Commands.runOnce (instead of swerve.runOnce) is a special case here
            // to allow this to run while other swerve commands (the default driving
            // command, for example) run. This is usually a horrible idea and shouldn't
            // be used outside of special cases like this.

            // The .ignoringDisable makes sure slow mode won't get stuck on or off if
            // the robot is disabled.
            Commands.runOnce(() -> manager.swerveSubsystem.setSlowMode(true)).ignoringDisable(true)
        ).onFalse(
            Commands.runOnce(() -> manager.swerveSubsystem.setSlowMode(false)).ignoringDisable(true)
        );

        Controls.zeroGryoscope.onTrue(
            // Similar comment on Commands.runOnce as slow mode above
            Commands.runOnce(() -> manager.swerveSubsystem.resetHeading())
        );

        Controls.robotRelative.onTrue(
            // Similar comment on Commands.runOnce and ignoringDisable as slow mode above
            Commands.runOnce(() -> manager.swerveSubsystem.setRobotRelative(true)).ignoringDisable(true)
        ).onFalse(
            Commands.runOnce(() -> manager.swerveSubsystem.setRobotRelative(false)).ignoringDisable(true)
        );

        Controls.snapNorth.whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(0),
                () -> Controls.driveTranslateX.getAsDouble(),
                () -> Controls.driveTranslateY.getAsDouble()
            )
        );

        Controls.snapSouth.whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(180),
                () -> Controls.driveTranslateX.getAsDouble(),
                () -> Controls.driveTranslateY.getAsDouble()
            )
        );

        Controls.snapEast.whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(270),
                () -> Controls.driveTranslateX.getAsDouble(),
                () -> Controls.driveTranslateY.getAsDouble()
            )
        );

        Controls.snapWest.whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(90),
                () -> Controls.driveTranslateX.getAsDouble(),
                () -> Controls.driveTranslateY.getAsDouble()
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
    };



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoManager.getAutonomousCommand();
    }
}
