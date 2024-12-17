package com.frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.lib.team8592.MatchMode;

import edu.wpi.first.wpilibj2.command.*;

public class SubsystemManager extends SubsystemBase {
    private SwerveSubsystem swerveSubsystem;

    private List<NewtonSubsystem> activeSubystems = new ArrayList<>();

    public SubsystemManager(boolean logToShuffleboard) {
        this.swerveSubsystem = new SwerveSubsystem(logToShuffleboard);

        this.activeSubystems = List.of(
            // Add all active subsystems here
            swerveSubsystem
        );

        this.activeSubystems.forEach(s -> {
            s.enableSubsystem(true);
            s.initializeLogger();
        });
    }

    public Command onAutonomousInitCommand() {
        NewtonSubsystem[] subs = new NewtonSubsystem[activeSubystems.size()];
        for (int i = 0; i < activeSubystems.size(); i++) {
            subs[i] = activeSubystems.get(i);
        }

        return Commands.runOnce(() -> {
            activeSubystems.forEach(s -> s.onInit(MatchMode.AUTONOMOUS));
        }, subs);
    }

    public void onInit(MatchMode mode) {
        NewtonSubsystem[] subs = new NewtonSubsystem[activeSubystems.size()];
        for (int i = 0; i < activeSubystems.size(); i++) {
            subs[i] = activeSubystems.get(i);
        }

        activeSubystems.forEach(s -> s.onInit(mode));
    }

    public void onRobotInit() {
        NewtonSubsystem[] subs = new NewtonSubsystem[activeSubystems.size()];
        for (int i = 0; i < activeSubystems.size(); i++) {
            subs[i] = activeSubystems.get(i);
        }

        activeSubystems.forEach(s -> s.onRobotInit());
    }

    @Override
    public void periodic() {
        this.activeSubystems.forEach(s -> {
            s.periodicLogs();
            s.periodicOutputs();
        });
    }

    // ==================================== \\
    // Add all getSubsystem() methods below \\
    // ==================================== \\

    public SwerveSubsystem getSwerve() {
        return this.swerveSubsystem;
    }
}