// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.CONFIG;
import lib.team1731.*;
import lib.team1731.field.*;
import lib.team8592.logging.SmartLogger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private SmartLogger logger = new SmartLogger("Config");
    private Command autonomousCommand = Commands.none();

    private RobotContainer robotContainer;

    public static MatchMode MODE = MatchMode.DISABLED;
    
    public static final RobotClock CLOCK = new RobotClock();
    public static final FieldLayout FIELD = new ReefscapeFieldLayout();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
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

        Robot.FIELD.logToShuffleboard(Robot.isSimulation());
        this.robotContainer = new RobotContainer();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run(); 
        Controls.logControls();
        Robot.CLOCK.update();

        logger.log("Robot Type", RobotSelector.getRobot());
        logger.log("Match Mode", Robot.MODE);
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        this.updateMode(MatchMode.DISABLED);
    }

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        this.autonomousCommand = robotContainer.getAutonomousCommand();

        this.updateMode(MatchMode.AUTONOMOUS);

        if (autonomousCommand != null) {
            this.autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            this.autonomousCommand.cancel();
        }

        this.updateMode(MatchMode.TELEOP);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        this.updateMode(MatchMode.TEST);
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}

    private void updateMode(MatchMode mode) {
        Robot.MODE = mode;
        this.robotContainer.onModeInit(MODE).schedule();
    }
}
