package frc.robot.subsystems;

import java.util.*;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.RobotSelector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOKrakenX60;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.ctreswerve.TunerConstants;
import frc.robot.subsystems.vision.*;
import lib.team1731.MatchMode;

public class SubsystemManager extends SubsystemBase {
    public SwerveSubsystem swerve;
    public VisionSubsystem vision;
    public Elevator elevator;

    private List<Subsystem> activeSubystems = new ArrayList<>();

    public SubsystemManager() {
        switch(RobotSelector.getRobot()) {
            case SIM_BOT: // Robot for simulation
                this.swerve = new SwerveSubsystem(
                    new SwerveIOCTRE<TunerConstants>(TunerConstants.class) // CTRE Swerve works well in simulation
                );

                this.vision = new VisionSubsystem(
                    new CameraIOSim(Constants.VISION.CAM_NAME, Constants.VISION.CAMERA_OFFSET)
                );

                this.elevator = new Elevator(new ElevatorIOSim());
                break;
            case DEV_BOT: // Development robot
                this.swerve = new SwerveSubsystem(
                    new SwerveIOCTRE<TunerConstants>(TunerConstants.class)
                );

                this.vision = new VisionSubsystem(
                    new CameraIOArducam(Constants.VISION.CAM_NAME, Constants.VISION.CAMERA_OFFSET)
                );

                this.elevator = new Elevator(new ElevatorIOKrakenX60(Ports.ELEVATOR_CAN_ID, false));
                break;
            case COMP_BOT: // Main robot for competition
            // Note - Fall through intentional
            default:
                this.swerve = new SwerveSubsystem(
                    new SwerveIOCTRE<TunerConstants>(TunerConstants.class)
                );

                this.vision = new VisionSubsystem(
                    new CameraIOArducam(Constants.VISION.CAM_NAME, Constants.VISION.CAMERA_OFFSET)
                );

                this.elevator = new Elevator(new ElevatorIOKrakenX60(Ports.ELEVATOR_CAN_ID, false));
                break;
        }

        this.activeSubystems = List.of(
            // Add all active subsystems here
            swerve,
            vision,
            elevator
        );

        this.activeSubystems.forEach(s -> {
            s.enableSubsystem(true);
            // s.initializeLogger();
        });
    }

    public Command onModeInitCommand(MatchMode mode) {
        Subsystem[] subs = new Subsystem[activeSubystems.size()];
        for (int i = 0; i < activeSubystems.size(); i++) {
            subs[i] = activeSubystems.get(i);
        }

        return Commands.runOnce(() -> {
            activeSubystems.forEach(s -> s.onModeInit(mode));
        }, subs);
    }

    public List<Subsystem> getAllSubsystemsAsList() {
        return activeSubystems;
    }

    public Subsystem[] getAllSubsystemsAsArray() {
        Subsystem[] subsystems = new Subsystem[activeSubystems.size()];
        for (int i = 0; i < activeSubystems.size(); i++) {
            subsystems[i] = activeSubystems.get(i);
        }
        return subsystems;
    }

    @Override
    public void periodic() {
       
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