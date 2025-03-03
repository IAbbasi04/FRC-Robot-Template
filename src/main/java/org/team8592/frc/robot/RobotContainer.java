// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team8592.frc.robot;

import org.team8592.frc.robot.Controls.ControlSets;
import org.team8592.frc.robot.commands.NewtonCommands;
import org.team8592.frc.robot.commands.autonomous.*;
import org.team8592.frc.robot.commands.largecommands.LargeCommand;
import org.team8592.frc.robot.subsystems.SubsystemManager;
import org.team8592.frc.robot.subsystems.swerve.SwerveSubsystem;
import org.team8592.frc.robot.subsystems.vision.VisionSubsystem;
import org.team8592.lib.MatchMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;


public class RobotContainer {
    // The robot's subsystems
    private final SubsystemManager manager;
    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        this.manager = new SubsystemManager();

        this.swerve = manager.swerve;
        this.vision = manager.vision;

        // this.swerve = new SwerveSubsystem(new SwerveIOCTRE());

        Controls.applyControlSet(ControlSets.DUAL_DRIVER);

        passSubsystems();
        configureBindings();
        configureDefaults();

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
        // swerve.setDefaultCommand(
        //     // () -> {
        //     // swerve.drive(swerve.processJoystickInputs(
        //     //     1,
        //     //     0,
        //     //     0
        //     //     // Controls.driveTranslateX.getAsDouble(),
        //     //     // Controls.driveTranslateY.getAsDouble(),
        //     //     // Controls.driveRotate.getAsDouble()
        //     // ), DriveModes.AUTOMATIC);
        //     // }
        //     () -> {
        //         SmartDashboard.putNumber("CLOCK AALLS", Robot.CLOCK.get());
        //     }
        // );

        // swerve.setDefaultCommand(swerve.commands.joystickDriveCommand(
        //     1, 
        //     0, 
        //     0
        // ));

        swerve.setDefaultCommand(swerve.run(() -> {
            SmartDashboard.putNumber("AAMMCCVVAAASDJKL", 155);
        }));

        // swerve.drive(new ChassisSpeeds(1, 0, 0));

        // swerve.setDefaultCommand(
        //     new DeferredCommand(() -> swerve.commands.joystickDriveCommand(
        //         Controls.driveTranslateX.getAsDouble(), 
        //         Controls.driveTranslateY.getAsDouble(), 
        //         Controls.driveRotate.getAsDouble()
        //     ), Set.of(swerve))
        // );

        vision.setDefaultCommand(NewtonCommands.updateOdometryWithVision(swerve, vision));
    }

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
            Commands.runOnce(() -> manager.swerve.setSlowMode(true)).ignoringDisable(true)
        ).onFalse(
            Commands.runOnce(() -> manager.swerve.setSlowMode(false)).ignoringDisable(true)
        );

        Controls.zeroGryoscope.onTrue(
            // Similar comment on Commands.runOnce as slow mode above
            Commands.runOnce(() -> manager.swerve.resetHeading())
        );

        Controls.robotRelative.onTrue(
            // Similar comment on Commands.runOnce and ignoringDisable as slow mode above
            Commands.runOnce(() -> manager.swerve.setRobotRelative(true)).ignoringDisable(true)
        ).onFalse(
            Commands.runOnce(() -> manager.swerve.setRobotRelative(false)).ignoringDisable(true)
        );

        Controls.snapNorth.whileTrue(
            swerve.commands.snapToAngleCommand(
                Rotation2d.fromDegrees(0),
                Controls.driveTranslateX,
                Controls.driveTranslateY
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.snapSouth.whileTrue(
            swerve.commands.snapToAngleCommand(
                Rotation2d.fromDegrees(180),
                Controls.driveTranslateX,
                Controls.driveTranslateY
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.snapEast.whileTrue(
            swerve.commands.snapToAngleCommand(
                Rotation2d.fromDegrees(270),
                Controls.driveTranslateX,
                Controls.driveTranslateY
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.snapWest.whileTrue(
            swerve.commands.snapToAngleCommand(
                Rotation2d.fromDegrees(90),
                Controls.driveTranslateX,
                Controls.driveTranslateY
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
    };

    public Command onModeInit(MatchMode mode) {
        return manager.onModeInitCommand(mode);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoManager.getAutonomousCommand();
    }
}