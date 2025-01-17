// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team8592.frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;

import org.team8592.lib.logging.LogUtils;
import org.team8592.lib.logging.LogUtils.WidgetProfile;
import org.team8592.frc.robot.Robot;
import org.team8592.frc.robot.commands.proxies.*;
import org.team8592.frc.robot.subsystems.SubsystemManager;

/**
 * General class for autonomous management (loading autos, sending the chooser, getting the
 * user-selected auto command, etc).
 */
public final class AutoManager {
    private static SendableChooser<AutoCommand> autoChooser;
    private static SubsystemManager manager;

    /**
     * Load all autos and broadcast the chooser.
     *<p>
     * * This is where programmers should add new autos.
     *
     * @apiNote This should be called on {@link Robot#robotInit()} only;
     * this function will have relatively long delays due to loading paths.
     */
    public static void prepare(SubsystemManager manager){
        AutoManager.manager = manager;
        AutoManager.autoChooser = LogUtils.createSendableChooserWithDefault(
            AutoCommand.getDefaultAuto(), 
            List.of(
                // Add all autos here
            )
        );

        LogUtils.addSendable("Autonomous Config", 
            autoChooser, 
            new WidgetProfile()
                .withPosition(4, 2)
                .withSize(3, 2)
        );
    }

    /**
     * Get the user-selected autonomous command as determined by {@link AutoManager#autoChooser}
     *
     * @return the command
     */
    public static Command getAutonomousCommand(){
        AutoCommand autoCommand = autoChooser.getSelected();
        manager.getSwerve().resetPose(autoCommand.getStartPose());
        return manager.onAutonomousInitCommand().andThen(
            // If we don't keep this command from registering as composed,
            // the code will crash if we try to run an auto twice without
            // restarting robot code.
            new MultiComposableCommand(autoCommand)
        );
    }

    private AutoManager() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
