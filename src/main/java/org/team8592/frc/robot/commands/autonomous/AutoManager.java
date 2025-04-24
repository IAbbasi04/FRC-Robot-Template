// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team8592.frc.robot.commands.autonomous;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import org.team8592.frc.robot.Robot;
import org.team8592.frc.robot.commands.proxies.*;
import org.team8592.frc.robot.subsystems.SubsystemManager;

/**
 * General class for autonomous management (loading autos, sending the chooser, getting the
 * user-selected auto command, etc).
 */
public final class AutoManager {
    private static SubsystemManager manager;
    public static void addSubsystems(SubsystemManager manager){
        AutoManager.manager = manager;
    }

    private static SendableChooser<AutoCommand> autoChooser;
    private static ArrayList<AutoCommand> autoCommands = new ArrayList<>();

    /**
     * Load all autos and broadcast the chooser.
     *<p>
     * * This is where programmers should add new autos.
     *
     * @apiNote This should be called on {@link Robot#robotInit()} only;
     * this function will have relatively long delays due to loading paths.
     */
    public static void prepare(){
        autoCommands = new ArrayList<>();


        // autoCommands.add(new ExampleAuto());
        // TODO: Add autos here

        autoChooser = new SendableChooser<>();
        
        autoChooser.setDefaultOption("DEFAULT - No auto", AutoCommand.none());
        for(AutoCommand c : autoCommands){
            autoChooser.addOption(
                c.getClass().getSimpleName()+(
                    c.getStartPose().equals(new Pose2d()) ? " (WARNING: NO START POSE)" : ""
                ), c
            );
        }
        
        Shuffleboard.getTab("Autonomous Config").add(autoChooser);
    }

    /**
     * Get the user-selected autonomous command as determined by {@link AutoManager#autoChooser}
     *
     * @return the command
     */
    public static Command getAutonomousCommand(){
        AutoCommand autoCommand = autoChooser.getSelected();
        return new ConditionalCommand(
            getAutonomousInitCommand().andThen(
                // If we don't keep this command from registering as composed,
                // the code will crash if we try to run an auto twice without
                // restarting robot code.
                new MultiComposableCommand(autoCommand)
            ), 
                getAutonomousInitCommand().andThen(
                    manager.swerve.runOnce(() -> manager.swerve.resetPose(
                        autoCommand.getStartPose(),
                        DriverStation.getAlliance().isPresent() && 
                            DriverStation.getAlliance().get() == Alliance.Red
                    )
                )
            ).andThen(
                new MultiComposableCommand(autoCommand)
            ),
            () -> autoCommand.getStartPose().equals(null)
        );
    }

    /**
     * Parallel command group that runs all subsystems' autonomous init commands.
     *
     * @return the command
     */
    private static Command getAutonomousInitCommand(){
        return new ParallelCommandGroup(
            manager.swerve.getStopCommand().alongWith(
                manager.swerve.resetHeading()
            )
        );
    }

    private AutoManager() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
