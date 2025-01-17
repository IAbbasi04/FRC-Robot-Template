// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team8592.frc.robot;


import org.littletonrobotics.junction.LoggedRobot;
import org.team8592.lib.MatchMode;
import org.team8592.lib.RobotClock;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import org.team8592.lib.field.FieldLayout;
import org.team8592.lib.logging.LogUtils;
import org.team8592.lib.logging.LogUtils.LogConstants;

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
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    public static MatchMode MODE = MatchMode.DISABLED;
    public static RobotClock CLOCK = new RobotClock();
    public static FieldLayout FIELD = FieldLayout.none();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        LogUtils.initialize(new LogConstants(
            Constants.LOGGER.GAME, 
            Constants.LOGGER.YEAR, 
            Constants.LOGGER.ROBOT, 
            Constants.LOGGER.TEAM
        ), isSimulation());

        this.robotContainer = new RobotContainer(!DriverStation.isFMSAttached());
        
        FIELD.logToShuffleboard(robotContainer.logToShuffleboard());
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
        Controls.logControlsToShuffleboard();
        CLOCK.update();
        LogUtils.logToSmartDashboard("Clock dt", CLOCK.dt());
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        MODE = MatchMode.DISABLED;
        this.robotContainer.runSubsystemsInit(MODE);
    }

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }

        MODE = MatchMode.AUTONOMOUS;
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
            autonomousCommand.cancel();
        }

        MODE = MatchMode.TELEOP;
        this.robotContainer.configureDefaults();
        this.robotContainer.runSubsystemsInit(MODE);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        MODE = MatchMode.TEST;
        this.robotContainer.removeDefaults();
        this.robotContainer.runSubsystemsInit(MODE);
        this.robotContainer.scheduleUnitTests();
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
}
