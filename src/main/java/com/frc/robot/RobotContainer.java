// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frc.robot;

import com.frc.robot.autonomous.AutoManager;
import com.frc.robot.commands.*;
import com.frc.robot.commands.proxies.NewtonWrapperCommand;
import com.frc.robot.subsystems.*;
import com.frc.robot.subsystems.SwerveSubsystem.DriveModes;
import com.frc.robot.unittest.UnitTestScheduler;
import com.lib.team8592.MatchMode;
import com.lib.team8592.logging.LogUtils;

import com.frc.robot.Controls.ControlSets;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;

public class RobotContainer {
    private SubsystemManager activeSubsystemsManager;
    private SwerveSubsystem swerve;

    private UnitTestScheduler testScheduler;

    private boolean logToShuffleboard = false;

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer(boolean logToShuffleboard) {
        this.activeSubsystemsManager = new SubsystemManager(logToShuffleboard);
        this.testScheduler = new UnitTestScheduler(activeSubsystemsManager);

        this.logToShuffleboard = logToShuffleboard;
        
        NewtonCommands.initialize(activeSubsystemsManager);
        NewtonWrapperCommand.initialize(activeSubsystemsManager);
        AutoManager.prepare(activeSubsystemsManager);

        Controls.initializeShuffleboardLogs(logToShuffleboard);

        // Add subsystems here
        swerve = activeSubsystemsManager.getSwerve();

        this.configureBindings(ControlSets.DUAL_DRIVER);
        this.configureDefaults();
        this.registerNamedCommands();

        this.activeSubsystemsManager.onRobotInit();
        LogUtils.addSendable(activeSubsystemsManager);
    }

    /**
     * Registers all named commands to be used as events in PathPlanner autos
     */
    private void registerNamedCommands() {
        // Register named commands for PathPlanner event markers
    }

    /**
     * Configure default commands for the subsystems
     */
    public void configureDefaults(){
        // Set the swerve's default command to drive with joysticks
        swerve.setDefaultCommand(swerve.run(() -> {
            swerve.drive(swerve.processJoystickInputs(
                Controls.driveTranslateX.getAsDouble(),
                Controls.driveTranslateY.getAsDouble(),
                Controls.driveRotate.getAsDouble()
            ), DriveModes.AUTOMATIC);
        }));

        activeSubsystemsManager.getIntake().setStopAsDefaultCommand();
    }

    public void removeDefaults() {
        swerve.removeDefaultCommand();
    }

    /**
     * Configure all button bindings
     *
     * @param controlSet the set of controls to use
     */
    private void configureBindings(ControlSets controlSet) {
        CommandScheduler.getInstance().getDefaultButtonLoop().clear();
        Controls.applyControlSet(controlSet);

        Controls.slowMode.onTrue(
            Commands.runOnce(() -> swerve.setSlowMode(true)).ignoringDisable(true)
        ).onFalse(
            Commands.runOnce(() -> swerve.setSlowMode(false)).ignoringDisable(true)
        );

        Controls.zeroGryoscope.onTrue(
            // Similar comment on Commands.runOnce as slow mode above
            Commands.runOnce(() -> swerve.resetHeading())
        );

        Controls.robotRelative.onTrue(
            // Similar comment on Commands.runOnce and ignoringDisable as slow mode above
            Commands.runOnce(() -> swerve.setRobotRelative(true)).ignoringDisable(true)
        ).onFalse(
            Commands.runOnce(() -> swerve.setRobotRelative(false)).ignoringDisable(true)
        );

        Controls.snapForward.whileTrue(
            NewtonCommands.swerveSnapToCommand(
                Rotation2d.fromDegrees(0),
                Controls.driveTranslateX,
                Controls.driveTranslateY
            )
        );

        Controls.snapBack.whileTrue(
            NewtonCommands.swerveSnapToCommand(
                Rotation2d.fromDegrees(180),
                Controls.driveTranslateX,
                Controls.driveTranslateY
            )
        );

        Controls.snapRight.whileTrue(
            NewtonCommands.swerveSnapToCommand(
                Rotation2d.fromDegrees(270),
                Controls.driveTranslateX,
                Controls.driveTranslateY
            )
        );

        Controls.snapLeft.whileTrue(
            NewtonCommands.swerveSnapToCommand(
                Rotation2d.fromDegrees(90),
                Controls.driveTranslateX,
                Controls.driveTranslateY
            )
        );

        Controls.getDriver().leftTrigger().whileTrue(
            activeSubsystemsManager.getIntake().run(() -> {
                activeSubsystemsManager.getIntake().setRollerVelocity(2000);
            })
        );

        Controls.getDriver().rightTrigger().whileTrue(
            activeSubsystemsManager.getIntake().run(() -> {
                activeSubsystemsManager.getIntake().setRollerVelocity(-2000);
            })
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoManager.getAutonomousCommand();
    }

    /**
     * Whether we want to log to shuffleboard
     */
    public boolean logToShuffleboard() {
        return logToShuffleboard;
    }

    /**
     * Runs the onInit() method for each active subsystem based on the given mode
     */
    public void runSubsystemsInit(MatchMode mode) {
        activeSubsystemsManager.onInit(mode);
    }

    public void scheduleUnitTests() {
        testScheduler.getUnitTestCommand();
        CommandScheduler.getInstance().schedule(
            testScheduler.getUnitTestCommand()
        );
    }
}
