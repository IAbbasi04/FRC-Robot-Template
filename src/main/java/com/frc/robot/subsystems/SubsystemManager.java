package com.frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.frc.robot.Suppliers;
import com.lib.team8592.MatchMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;

public class SubsystemManager extends SubsystemBase {
    private static SubsystemManager instance = null;
    public static SubsystemManager getInstance(boolean logToShuffleboard) {
        if (instance == null) instance = new SubsystemManager(logToShuffleboard);
        return instance;
    }

    private Superstructure superstructure;

    private Swerve swerve;
    private Pivot pivot;

    private List<NewtonSubsystem> activeSubystems = new ArrayList<>();

    private SubsystemManager(boolean logToShuffleboard) {
        this.superstructure = new Superstructure(logToShuffleboard);
        this.swerve = new Swerve(logToShuffleboard);
        // this.pivotSubsystem = new PivotSubsystem(logToShuffleboard);

        this.activeSubystems = List.of(
            // Add all active subsystems here
            superstructure,
            swerve
            // pivotSubsystem
        );

        this.activeSubystems.forEach(s -> {
            s.enableSubsystem(true);
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

        activeSubystems.forEach(s -> {
            s.onRobotInit();
            s.hasInit = true;
        });
    }

    @Override
    public void periodic() {
        this.activeSubystems.forEach(s -> {
            s.periodicLogs();
            s.periodicOutputs();
            if (Suppliers.robotIsReal.getAsBoolean()) {
                s.simulationPeriodic();
            }

            s.hasInit = true; // In case not already set
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SubsystemManager");
        activeSubystems.forEach(sub -> {
            builder.addBooleanProperty(sub.getName() + " Enabled", 
                sub::isEnabled, 
                (enable) -> {
                    sub.enableSubsystem(enable);
                }
            );
        });
    }

    // ==================================== \\
    // Add all getSubsystem() methods below \\
    // ==================================== \\

    public List<NewtonSubsystem> getAllSubsystemsAsList() {
        return activeSubystems;
    }

    public NewtonSubsystem[] getAllSubsystemsAsArray() {
        NewtonSubsystem[] subsystems = new NewtonSubsystem[activeSubystems.size()];
        for (int i = 0; i < activeSubystems.size(); i++) {
            subsystems[i] = activeSubystems.get(i);
        }
        return subsystems;
    }

    public Superstructure getSuperstructure() {
        return this.superstructure;
    }

    public Swerve getSwerve() {
        return this.swerve;
    }

    public Pivot getPivot() {
        return this.pivot;
    }
}