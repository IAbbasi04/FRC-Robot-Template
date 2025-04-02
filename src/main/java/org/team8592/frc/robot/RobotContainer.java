// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team8592.frc.robot;

import org.team8592.frc.robot.Controls.ControlSets;
import org.team8592.frc.robot.commands.SuperCommands;
import org.team8592.frc.robot.commands.autonomous.*;
import org.team8592.frc.robot.subsystems.SubsystemManager;
import org.team8592.frc.robot.subsystems.roller.RollerSubsystem;
import org.team8592.frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import org.team8592.frc.robot.subsystems.superstructure.shoulder.ShoulderSubsystem;
import org.team8592.frc.robot.subsystems.superstructure.wrist.WristSubsystem;
import org.team8592.frc.robot.subsystems.swerve.SwerveSubsystem;
import org.team8592.frc.robot.subsystems.vision.VisionSubsystem;
import org.team8592.lib.MatchMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;


public class RobotContainer {
    // The robot's subsystems
    private final SubsystemManager manager;
    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;
    private final RollerSubsystem roller;
    private final WristSubsystem wrist;
    private final ShoulderSubsystem shoulder;
    private final ElevatorSubsystem elevator;

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        this.manager = new SubsystemManager();

        this.swerve = manager.swerve;
        this.vision = manager.vision;
        this.roller = manager.roller;
        this.wrist = manager.wrist;
        this.shoulder = manager.shoulder;
        this.elevator = manager.elevator;

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
        roller.setStopAsDefaultCommand();
        wrist.setStopAsDefaultCommand();
        shoulder.setDefaultCommand(shoulder.setDegrees(() -> 90).ignoringDisable(false));
        elevator.setDefaultCommand(elevator.setInches(() -> 10).ignoringDisable(false));
    }

    /**
     * Configure all button bindings
     */
    private void configureBindings() {
        Controls.slowMode.onTrue(manager.swerve.setSlowMode(true))
            .onFalse(manager.swerve.setSlowMode(false));

        Controls.zeroGryoscope.onTrue(manager.swerve.resetHeading());

        Controls.robotRelative.onTrue(manager.swerve.setRobotRelative(true))
            .onFalse(manager.swerve.setRobotRelative(false));

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

        Controls.intake.whileTrue(roller.setPercentPower(0.5));
        Controls.score.whileTrue(roller.setPercentPower(-0.5));

        Controls.getDriver().y().whileTrue(wrist.setPercentPower(0.5));
        Controls.getDriver().a().whileTrue(wrist.setPercentPower(-0.5));
        // Controls.getDriver().y().whileTrue(wrist.setDegrees(90));
        // Controls.getDriver().a().whileTrue(wrist.setDegrees(-90));
        // Controls.getDriver().x().whileTrue(wrist.setDegrees(0));
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