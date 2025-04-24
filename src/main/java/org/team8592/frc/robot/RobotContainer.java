// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team8592.frc.robot;

import org.team8592.frc.robot.Controls.ControlSets;
import org.team8592.frc.robot.commands.SuperCommands;
import org.team8592.frc.robot.commands.autonomous.*;
import org.team8592.frc.robot.subsystems.SubsystemManager;
import org.team8592.frc.robot.subsystems.roller.RollerSubsystem;
import org.team8592.frc.robot.subsystems.superstructure.Superstructure;
import org.team8592.frc.robot.subsystems.superstructure.Superstructure.GamePiece;
import org.team8592.frc.robot.subsystems.superstructure.Superstructure.States;
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

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        this.manager = new SubsystemManager();

        this.swerve = manager.swerve;
        this.vision = manager.vision;
        this.roller = manager.roller;

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

        Controls.intake.whileTrue(
            // Check to see if we are in algae mode
            new ConditionalCommand( 
                
                // Spin rollers to intake algae
                manager.roller.setPercentPower(-0.3),

                // Check to see if we are in coral mode
                new ConditionalCommand( 

                    // Spin rollers to intake coral
                    manager.roller.setPercentPower(1.0),

                    // Check to see if we are in deep climb mode
                    new ConditionalCommand( 

                        // Run Deep climb intake if we are neither in coral or algae mode
                        // TODO - ADD ABILITY TO INTAKE CAGE
                        Commands.none(),

                        // Do nothing if we are in neither coral, algae, nor deep climb mode
                        Commands.none(), 

                        // If we are in climb mode
                        Suppliers.IS_CLIMB_MODE
                    ), 

                    // If we are in coral mode
                    Suppliers.IS_CORAL_MODE
                ), 

                // If we are in algae mode
                Suppliers.IS_ALGAE_MODE
            )
        );

        Controls.score.whileTrue(
            // Check to see if we are in algae mode
            new ConditionalCommand( 
                
                // Spin rollers to score algae
                manager.roller.setPercentPower(1.0),

                // Check to see if we are in coral mode
                new ConditionalCommand( 

                    // Check to see if we are in coral L1 scoring mode
                    new ConditionalCommand(

                        // Score at lowered speed for L1
                        manager.roller.setPercentPower(-0.5), 

                        // Spin rollers to score coral
                        manager.roller.setPercentPower(-1.0), 

                        // If we are in L1 mode
                        () -> Superstructure.appliedState.equals(States.L1)
                    ),

                    // Check to see if we are in deep climb mode
                    new ConditionalCommand( 

                        // Run deep climb rollers to outtake if we are in climb mode
                        // TODO - ADD ABILITY TO OUTTAKE CAGE
                        Commands.none(),

                        // Do nothing if we are in neither coral, algae, nor deep climb mode
                        Commands.none(), 

                        // If we are in climb mode
                        Suppliers.IS_CLIMB_MODE
                    ), 

                    // If we are in coral mode
                    Suppliers.IS_CORAL_MODE
                ), 

                // If we are in algae mode
                Suppliers.IS_ALGAE_MODE
            )
        );

        Controls.stow.onTrue(
            // Check to see if we are in algae mode
            new ConditionalCommand( 
                
                // Spin rollers to intake algae
                manager.roller.setPercentPower(-0.3),

                // Check to see if we are in coral mode
                new ConditionalCommand( 

                    // Spin rollers to intake coral
                    manager.roller.setPercentPower(1.0),

                    // Check to see if we are in deep climb mode
                    new ConditionalCommand( 
                        // Go back to coral stow mode and reset climber
                        Superstructure.setTargetGamePiece(GamePiece.CORAL).alongWith(
                            Superstructure.goToPosition(States.DEEP_CLIMB_CLEARANCE)
                            // TODO - ADD ABILITY TO RESET CLIMBER
                        ).andThen(
                            Superstructure.goToPosition(States.STOW)
                        ),

                        // Do nothing if we are in neither coral, algae, nor deep climb mode
                        Commands.none(), 

                        // If we are in climb mode
                        Suppliers.IS_CLIMB_MODE
                    ), 

                    // If we are in coral mode
                    Suppliers.IS_CORAL_MODE
                ), 

                // If we are in algae mode
                Suppliers.IS_ALGAE_MODE
            )
        );

        Controls.goToTargetPosition.onTrue(Superstructure.goToTargetPosition());

        Controls.setCoralMode.onTrue(Superstructure.setTargetGamePiece(GamePiece.CORAL));
        Controls.setAlgaeMode.onTrue(Superstructure.setTargetGamePiece(GamePiece.ALGAE));
        Controls.setDeepClimbMode.onTrue(Superstructure.setTargetGamePiece(GamePiece.CAGE));

        Controls.primeL1.onTrue(Superstructure.setTargetPosition(States.L1));
        Controls.primeL2.onTrue(Superstructure.setTargetPosition(States.L2));
        Controls.primeL3.onTrue(Superstructure.setTargetPosition(States.L3));
        Controls.primeL4.onTrue(Superstructure.setTargetPosition(States.L4));

        Controls.primeProcessor.onTrue(Superstructure.setTargetPosition(States.PROCESSOR));
        Controls.primeL2Algae.onTrue(Superstructure.setTargetPosition(States.L2_ALGAE));
        Controls.primeL3Algae.onTrue(Superstructure.setTargetPosition(States.L3_ALGAE));
        Controls.primeNet.onTrue(Superstructure.setTargetPosition(States.NET));

        Controls.autoLockLeftBranch.whileTrue(
            // TODO - ADD ABILITY TO AUTO LOCK
            Commands.none()
        );

        Controls.autoLockRightBranch.whileTrue(
            // TODO - ADD ABILITY TO AUTO LOCK
            Commands.none()
        );

        Controls.manualDeepClimbUp.whileTrue(
            // TODO - ADD ABILITY TO MOVE DEEP CLIMB UP
            Commands.none()
        );

        Controls.manualDeepClimbDown.whileTrue(
            // TODO - ADD ABILITY TO MOVE DEEP CLIMB DOWN
            Commands.none()
        );

        Controls.deepClimbInitialize.onTrue(
            // TODO - ADD ABILITY TO INITIALIZE DEEP CLIMB
            Commands.none()
        );
    };

    public Command onModeInit(MatchMode mode) {
        return manager.onModeInit(mode);
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