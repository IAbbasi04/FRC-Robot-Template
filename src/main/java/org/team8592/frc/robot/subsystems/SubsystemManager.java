package org.team8592.frc.robot.subsystems;

import java.util.*;

import org.team8592.frc.robot.subsystems.swerve.SwerveSubsystem;
import org.team8592.frc.robot.subsystems.vision.VisionSubsystem;
import org.team8592.lib.MatchMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;

public class SubsystemManager extends SubsystemBase {
    public SwerveSubsystem swerve;
    public VisionSubsystem vision;

    private List<NewtonSubsystem<?>> activeSubystems = new ArrayList<>();

    public SubsystemManager(boolean logToShuffleboard) {
        this.swerve = new SwerveSubsystem(logToShuffleboard);
        this.vision = new VisionSubsystem(logToShuffleboard);

        this.activeSubystems = List.of(
            // Add all active subsystems here
            swerve,
            vision
        );

        this.activeSubystems.forEach(s -> {
            s.enableSubsystem(true);
            s.initializeLogger();
        });
    }

    public void onModeInit(MatchMode mode) {
        NewtonSubsystem<?>[] subs = new NewtonSubsystem<?>[activeSubystems.size()];
        for (int i = 0; i < activeSubystems.size(); i++) {
            subs[i] = activeSubystems.get(i);
        }

        activeSubystems.forEach(s -> s.onModeInit(mode));
    }

    public void onRobotInit() {
        NewtonSubsystem<?>[] subs = new NewtonSubsystem<?>[activeSubystems.size()];
        for (int i = 0; i < activeSubystems.size(); i++) {
            subs[i] = activeSubystems.get(i);
        }

        activeSubystems.forEach(s -> s.onRobotInit());
    }

    public Command onModeInitCommand(MatchMode mode) {
        NewtonSubsystem<?>[] subs = new NewtonSubsystem<?>[activeSubystems.size()];
        for (int i = 0; i < activeSubystems.size(); i++) {
            subs[i] = activeSubystems.get(i);
        }

        return Commands.runOnce(() -> {
            activeSubystems.forEach(s -> s.onModeInit(mode));
        }, subs);
    }

    public Command onRobotInitCommand() {
        NewtonSubsystem<?>[] subs = new NewtonSubsystem<?>[activeSubystems.size()];
        for (int i = 0; i < activeSubystems.size(); i++) {
            subs[i] = activeSubystems.get(i);
        }

        return Commands.runOnce(() -> {
            activeSubystems.forEach(s -> s.onRobotInit());
        }, subs);
    }

    public List<NewtonSubsystem<?>> getAllSubsystemsAsList() {
        return activeSubystems;
    }

    public NewtonSubsystem<?>[] getAllSubsystemsAsArray() {
        NewtonSubsystem<?>[] subsystems = new NewtonSubsystem<?>[activeSubystems.size()];
        for (int i = 0; i < activeSubystems.size(); i++) {
            subsystems[i] = activeSubystems.get(i);
        }
        return subsystems;
    }

    @Override
    public void periodic() {
        this.activeSubystems.forEach(s -> {
            s.periodicTelemetry();
            s.periodicOutputs();
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SubsystemManager");
        activeSubystems.forEach(sub -> {
            builder.addBooleanProperty(sub.getName() + "/Enabled", 
                sub::isEnabled, 
                (enable) -> {
                    sub.enableSubsystem(enable);
                }
            );
        });
    }
}