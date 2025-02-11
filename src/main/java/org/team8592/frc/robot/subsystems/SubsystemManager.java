package org.team8592.frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.team8592.lib.MatchMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;

public class SubsystemManager extends SubsystemBase {
    // private SwerveSubsystem swerveSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private List<NewtonSubsystem> activeSubystems = new ArrayList<>();

    public SubsystemManager(boolean logToShuffleboard) {
        // this.swerveSubsystem = new SwerveSubsystem(logToShuffleboard);
        this.intakeSubsystem = new IntakeSubsystem(logToShuffleboard);

        this.activeSubystems = List.of(
            // Add all active subsystems here
            // swerveSubsystem,
            intakeSubsystem
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

    // public SwerveSubsystem getSwerve() {
        // return this.swerveSubsystem;
    // }

    public IntakeSubsystem getIntake() {
        return this.intakeSubsystem;
    }
}