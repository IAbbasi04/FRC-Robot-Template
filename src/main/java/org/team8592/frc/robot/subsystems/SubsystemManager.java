package org.team8592.frc.robot.subsystems;

import java.util.*;

import org.team8592.frc.robot.*;
import org.team8592.frc.robot.subsystems.roller.*;
import org.team8592.frc.robot.subsystems.superstructure.elevator.ElevatorIOKrakenX60;
import org.team8592.frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import org.team8592.frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import org.team8592.frc.robot.subsystems.superstructure.shoulder.ShoulderIOKrakenX60;
import org.team8592.frc.robot.subsystems.superstructure.shoulder.ShoulderIOSim;
import org.team8592.frc.robot.subsystems.superstructure.shoulder.ShoulderSubsystem;
import org.team8592.frc.robot.subsystems.superstructure.wrist.WristIOKrakenX60;
import org.team8592.frc.robot.subsystems.superstructure.wrist.WristIOSim;
import org.team8592.frc.robot.subsystems.superstructure.wrist.WristSubsystem;
import org.team8592.frc.robot.subsystems.swerve.*;
import org.team8592.frc.robot.subsystems.vision.*;
import org.team8592.lib.MatchMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;

public class SubsystemManager extends SubsystemBase {
    public SwerveSubsystem swerve;
    public VisionSubsystem vision;
    public RollerSubsystem roller;
    public WristSubsystem wrist;
    public ShoulderSubsystem shoulder;
    public ElevatorSubsystem elevator;

    private List<NewtonSubsystem> activeSubystems = new ArrayList<>();

    public SubsystemManager() {
        this.swerve = new SwerveSubsystem(new SwerveIOCTRE()); // Default amongst all bots

        switch(RobotSelector.getRobot()) {
            case SIM_BOT: // Robot for simulation
                this.roller = new RollerSubsystem(new RollerIOSim());
                this.wrist = new WristSubsystem(new WristIOSim());
                this.shoulder = new ShoulderSubsystem(new ShoulderIOSim());
                this.elevator = new ElevatorSubsystem(new ElevatorIOSim());
                this.vision = new VisionSubsystem(
                    new CameraIOSim(getName(), Constants.VISION.CAMERA_OFFSET)
                );
                break;

            case DEV_BOT: // Development robot
                this.roller = new RollerSubsystem(new RollerIOKrakenX60(Ports.ROLLER_CAN_ID));
                this.wrist = new WristSubsystem(new WristIOKrakenX60(Ports.WRIST_CAN_ID));
                this.shoulder = new ShoulderSubsystem(new ShoulderIOKrakenX60(Ports.SHOULDER_CAN_ID));
                this.elevator = new ElevatorSubsystem(
                    new ElevatorIOKrakenX60(
                        Ports.FRONT_ELEVATOR_CAN_ID, 
                        false, 
                        Ports.BACK_ELEVATOR_CAN_ID, 
                        true
                    )
                );

                this.vision = new VisionSubsystem(
                    new CameraIOArducam(
                        getName(),
                        Constants.VISION.CAMERA_OFFSET
                    )
                );
                break;

            case COMP_BOT: // Main robot for competition
            // Note - Fall through intentional
            default:
                this.roller = new RollerSubsystem(new RollerIOKrakenX60(Ports.ROLLER_CAN_ID));
                this.wrist = new WristSubsystem(new WristIOKrakenX60(Ports.WRIST_CAN_ID));
                this.shoulder = new ShoulderSubsystem(new ShoulderIOKrakenX60(Ports.SHOULDER_CAN_ID));
                this.elevator = new ElevatorSubsystem(
                    new ElevatorIOKrakenX60(
                        Ports.FRONT_ELEVATOR_CAN_ID, 
                        false, 
                        Ports.BACK_ELEVATOR_CAN_ID, 
                        true
                    )
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
            swerve,
            vision,
            roller,
            wrist,
            shoulder
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
        // this.activeSubystems.forEach(s -> {
        //     s.periodicTelemetry();
        //     s.periodicOutputs();
        // });
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