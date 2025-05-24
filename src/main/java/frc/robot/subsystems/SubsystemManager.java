package frc.robot.subsystems;

import java.util.*;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.*;

import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.gripper.*;
import frc.robot.subsystems.roller.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.ctreswerve.TunerConstants;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.wrist.*;

import lib.team1731.MatchMode;
import lib.team8592.hardware.motor.PortConfig;

public class SubsystemManager extends SubsystemBase {
    public SwerveSubsystem swerve;
    public VisionSubsystem vision;
    public ElevatorSubsystem elevator;
    public GripperSubsystem gripper;
    public WristSubsystem wrist;
    public RollerSubsystem roller;
    public ClimberSubsystem climber;

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

                this.elevator = new ElevatorSubsystem(new ElevatorIOSim());
                this.gripper = new GripperSubsystem(new GripperIOSim());
                this.wrist = new WristSubsystem(new WristIOSim());
                this.roller = new RollerSubsystem(new RollerIOSim());
                this.climber = new ClimberSubsystem(new ClimberIOSim());
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

                this.elevator = new ElevatorSubsystem(
                    new ElevatorIOTalonFX(new PortConfig(Ports.ELEVATOR_CAN_ID))
                );

                this.gripper = new GripperSubsystem(
                    new GripperIOTalonFX(new PortConfig(Ports.GRIPPER_CAN_ID))
                );

                this.wrist = new WristSubsystem(
                    new WristIOTalonFX(new PortConfig(Ports.WRIST_CAN_ID))
                );

                this.roller = new RollerSubsystem(
                    new RollerIOTalonFX(new PortConfig(Ports.ROLLER_CAN_ID))
                );

                this.climber = new ClimberSubsystem(
                    new ClimberIOTalonFX(new PortConfig(Ports.CLIMBER_CAN_ID))
                );
                break;
        }

        this.activeSubystems = List.of(
            // Add all active subsystems here
            swerve,
            vision,
            elevator,
            gripper,
            wrist,
            climber
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
        }, subs).ignoringDisable(mode.equals(MatchMode.DISABLED));
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