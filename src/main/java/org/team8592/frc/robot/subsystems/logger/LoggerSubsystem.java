package org.team8592.frc.robot.subsystems.logger;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.team8592.frc.robot.Robot;
import org.team8592.frc.robot.subsystems.NewtonSubsystem;
import org.team8592.lib.MatchMode;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import org.team8592.frc.robot.Constants.*;

public class LoggerSubsystem extends NewtonSubsystem {
    public LoggerSubsystem() {
        Logger.recordMetadata("Game", CONFIG.GAME);
        Logger.recordMetadata("Year", CONFIG.YEAR);
        Logger.recordMetadata("Robot", CONFIG.ROBOT);
        Logger.recordMetadata("Team", CONFIG.TEAM);

        if (Robot.isReal()) { // If running on a real robot
            String time = DateTimeFormatter.ofPattern("yy-MM-dd_HH-mm-ss").format(LocalDateTime.now());
            String path = "/U/"+time+".wpilog";
            Logger.addDataReceiver(new WPILOGWriter(path)); // Log to a USB stick
            LoggedPowerDistribution.getInstance(1, ModuleType.kRev); // Enables power distribution logging
        }
        
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

        Logger.start();
    }

    @Override
    public void onModeInit(MatchMode mode) {

    }

    @Override
    public void periodicTelemetry() {

    }

    @Override
    public void stop() {

    }
}