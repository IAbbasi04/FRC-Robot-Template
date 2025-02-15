package org.team8592.frc.robot.subsystems.logger;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.team8592.frc.robot.Robot;
import org.team8592.frc.robot.RobotConstants;
import org.team8592.frc.robot.subsystems.NewtonSubsystem;
import org.team8592.lib.MatchMode;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class LoggerSubsystem extends NewtonSubsystem {
    public LoggerSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);

        Logger.recordMetadata("Game", LoggerConstants.GAME);
        Logger.recordMetadata("Year", LoggerConstants.YEAR);
        Logger.recordMetadata("Robot", LoggerConstants.ROBOT);
        Logger.recordMetadata("Team", LoggerConstants.TEAM);

        if (Robot.isReal()) { // If running on a real robot
            String time = DateTimeFormatter.ofPattern("yy-MM-dd_HH-mm-ss").format(LocalDateTime.now());
            String path = "/U/"+time+".wpilog";
            Logger.addDataReceiver(new WPILOGWriter(path)); // Log to a USB stick
            LoggedPowerDistribution.getInstance(1, ModuleType.kRev); // Enables power distribution logging
        }
        
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.start();

        Logger.recordOutput("Testing I Guess", RobotConstants.getRobot());
    }

    @Override
    public void periodic() {

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