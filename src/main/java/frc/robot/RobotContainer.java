// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Controls.ControlSets;
import frc.robot.autonomous.*;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import lib.MatchMode;

/**
 * Integration class that takes inputs from subystems and controls and applies them to the robot
 */
public class RobotContainer {
    private final SubsystemManager manager;
    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;

    private final AutoLoader autoLoader;

    public RobotContainer() {
        this.manager = new SubsystemManager();
        this.autoLoader = new AutoLoader(manager);

        this.swerve = manager.swerve;
        this.vision = manager.vision;

        Controls.applyControlSet(ControlSets.DUAL_DRIVER);

        this.configureNamedCommands();
        this.configureControls();
        this.configureDefaults();
    }

    private void configureNamedCommands() {
        // NamedCommands.registerCommand("Example", exampleCommand());
        // TODO - Add named commands to use during PathPlanner autos
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
    private void configureControls() {
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
        return autoLoader.loadSelectedAuto();
    }
}