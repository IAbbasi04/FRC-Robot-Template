package org.team8592.frc.robot.subsystems;

import java.util.*;

import org.team8592.frc.robot.Constants;
import org.team8592.frc.robot.RobotSelector;
import org.team8592.frc.robot.subsystems.logger.*;
import org.team8592.frc.robot.subsystems.swerve.*;
import org.team8592.frc.robot.subsystems.vision.*;
import org.team8592.lib.MatchMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;

public class SubsystemManager extends SubsystemBase {
    public SwerveSubsystem swerve;
    public VisionSubsystem vision;
    public LoggerSubsystem logger;

    private List<NewtonSubsystem> activeSubystems = new ArrayList<>();

    public SubsystemManager() {
        this.logger = new LoggerSubsystem();

        switch(RobotSelector.getRobot()) {
            case PRAC_BOT:
                this.swerve = new SwerveSubsystem(
                    new SwerveIOCTRE()
                );

                this.vision = new VisionSubsystem(
                    new CameraIOArducam(
                        getName(),
                        Constants.VISION.CAMERA_OFFSET
                    )
                );
                break;
            case SIM_BOT:
                this.swerve = new SwerveSubsystem(
                    new SwerveIOCTRE() 
                );

                this.vision = new VisionSubsystem(
                    new CameraIOSim(
                        getName(), 
                        Constants.VISION.CAMERA_OFFSET
                    )
                );
                break;
            case COMP_BOT: // Fall through intentional
            default:
                this.swerve = new SwerveSubsystem(
                    new SwerveIOCTRE() 
                );

                this.vision = new VisionSubsystem(
                    new CameraIOArducam(
                        getName(), 
                        Constants.VISION.CAMERA_OFFSET
                    )
                );
                break;
        }

        this.activeSubystems = List.of(
            // Add all active subsystems here
            logger,
            swerve
            // ,
            // vision
        );

        this.activeSubystems.forEach(s -> {
            s.enableSubsystem(true);
            // s.initializeLogger();
        });
    }

    public Command onModeInitCommand(MatchMode mode) {
        NewtonSubsystem[] subs = new NewtonSubsystem[activeSubystems.size()];
        for (int i = 0; i < activeSubystems.size(); i++) {
            subs[i] = activeSubystems.get(i);
        }

        return Commands.runOnce(() -> {
            activeSubystems.forEach(s -> s.onModeInit(mode));
        }, subs);
    }

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