// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1731.frc.robot;

import org.team1731.frc.robot.Controls.ControlSets;
import org.team1731.frc.robot.commands.SuperCommands;
import org.team1731.frc.robot.commands.autonomous.*;
import org.team1731.frc.robot.subsystems.SubsystemManager;
import org.team1731.frc.robot.subsystems.swerve.SwerveSubsystem;
import org.team1731.frc.robot.subsystems.vision.VisionSubsystem;
import org.team1731.lib.MatchMode;

import edu.wpi.first.math.geometry.Rotation2d;
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

        Controls.applyControlSet(ControlSets.DUAL_DRIVER);
        
        passSubsystems();
        configureNamedCommands();
        configureBindings();
        configureDefaults();

        AutoManager.prepare();
    }

    private void configureNamedCommands() {
        // TODO - Add named commands to use during PathPlanner autos
    }

    /**
     * Pass subsystems everywhere they're needed
     */
    private void passSubsystems(){
        AutoManager.addSubsystems(manager);
        AutoCommand.addSubsystems(manager);
    }

    /**
     * Configure default commands for the subsystems
     */
    private void configureDefaults(){
        swerve.setDefaultCommand(swerve.joystickDrive(
            Controls.driveTranslateX, 
            Controls.driveTranslateY, 
            Controls.driveRotate
        ));

        vision.setDefaultCommand(SuperCommands.updateOdometryWithVision(swerve, vision));
    }

    /**
     * Configure all button bindings
     */
    private void configureBindings() {
        Controls.slowMode.onTrue(manager.swerve.setSlowMode(true).ignoringDisable(true))
            .onFalse(manager.swerve.setSlowMode(false).ignoringDisable(true));

        Controls.zeroGryoscope.onTrue(manager.swerve.resetHeading());

        Controls.robotRelative.onTrue(manager.swerve.setRobotRelative(true).ignoringDisable(true))
            .onFalse(manager.swerve.setRobotRelative(false).ignoringDisable(true));

        Controls.snapNorth.whileTrue(
            swerve.snapToAngle(
                Rotation2d.fromDegrees(0),
                Controls.driveTranslateX,
                Controls.driveTranslateY
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.snapSouth.whileTrue(
            swerve.snapToAngle(
                Rotation2d.fromDegrees(180),
                Controls.driveTranslateX,
                Controls.driveTranslateY
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.snapEast.whileTrue(
            swerve.snapToAngle(
                Rotation2d.fromDegrees(270),
                Controls.driveTranslateX,
                Controls.driveTranslateY
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.snapWest.whileTrue(
            swerve.snapToAngle(
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