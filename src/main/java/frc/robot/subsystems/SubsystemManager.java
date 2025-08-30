package frc.robot.subsystems;

import java.util.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.rollers.RollerSubsystem;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.ctre.BaseTunerConstants;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.wrist.WristSubsystem;
import lib.MatchMode;
import lib.subsystem.BaseSubsystem;
import lib.subsystem.SubsystemList;

/**
 * Class that handles all active and inactive subsystems on the robot
 */
public class SubsystemManager extends SubsystemBase {
    public SwerveSubsystem<BaseTunerConstants> swerve;
    public VisionSubsystem vision;
    public RollerSubsystem rollers;
    public PivotSubsystem pivot;
    public WristSubsystem wrist;
    public ElevatorSubsystem elevator;

    private SubsystemList activeSubsystems;

    public SubsystemManager() {
        this.swerve = new SwerveSubsystem<>();
        this.vision = new VisionSubsystem();
        this.elevator = new ElevatorSubsystem();
        this.wrist = new WristSubsystem();
        this.pivot = new PivotSubsystem();
        this.rollers = new RollerSubsystem();

        this.activeSubsystems = new SubsystemList(
            // Add all active subsystems here
            swerve,
            vision,
            elevator,
            wrist,
            pivot,
            rollers
        );

        this.activeSubsystems.forEach(s -> {
            s.enableSubsystem(true);
        });
    }

    /**
     * Returns a command that runs when the robot enters a specific match mode
     */
    public Command onModeInitCommand(MatchMode mode) {
        BaseSubsystem[] subs = new BaseSubsystem[activeSubsystems.size()];
        for (int i = 0; i < activeSubsystems.size(); i++) {
            subs[i] = activeSubsystems.get(i);
        }

        return Commands.runOnce(() -> {
            activeSubsystems.forEach(s -> s.onModeInit(mode));
        }, subs);
    }

    /**
     * Returns all active subsystems in the form of a list
     */
    public List<BaseSubsystem> getAllSubsystemsAsList() {
        return activeSubsystems;
    }

    /**
     * Returns all active subsystem in the form of an array
     */
    public BaseSubsystem[] getAllSubsystemsAsArray() {
        BaseSubsystem[] subsystems = new BaseSubsystem[activeSubsystems.size()];
        for (int i = 0; i < activeSubsystems.size(); i++) {
            subsystems[i] = activeSubsystems.get(i);
        }
        return subsystems;
    }

    @Override
    public void periodic() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SubsystemManager");
        activeSubsystems.forEach(sub -> {
            builder.addBooleanProperty(sub.getName() + "/Enabled", 
                sub::isEnabled, 
                (enable) -> {
                    sub.enableSubsystem(enable);
                }
            );
        });
    }
}