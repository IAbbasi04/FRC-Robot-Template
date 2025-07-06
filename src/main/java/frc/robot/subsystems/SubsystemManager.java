package frc.robot.subsystems;

import java.util.*;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.config.RobotSelector;
import frc.robot.subsystems.roller.coral.*;
import frc.robot.subsystems.grappler.*;
import frc.robot.subsystems.roller.algae.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.ctre.BaseTunerConstants;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.wrist.*;

import lib.MatchMode;
import lib.hardware.motor.PortConfig;
import lib.subsystem.*;

/**
 * Class that handles all active and inactive subsystems on the robot
 */
public class SubsystemManager extends SubsystemBase {
    public SwerveSubsystem swerve;
    public VisionSubsystem vision;

    public CoralRollerSubsystem coralIntake;
    public AlgaeRollerSubsystem algaeIntake;

    public WristSubsystem wrist;

    public GrapplerSubsystem grappler;

    private SubsystemList activeSubsystems;

    public SubsystemManager() {
        switch(RobotSelector.getRobot()) {
            case SIM_BOT: // Robot for simulation
                this.swerve = new SwerveSubsystem(
                    new SwerveIOCTRE<BaseTunerConstants>(BaseTunerConstants.class) // CTRE Swerve works well in simulation
                );

                this.vision = new VisionSubsystem(
                    new CameraIOSim(VisionConstants.CAM_NAME, VisionConstants.CAMERA_OFFSET)
                );

                this.coralIntake = new CoralRollerSubsystem(new CoralRollerIOSim());
                this.algaeIntake = new AlgaeRollerSubsystem(new AlgaeRollerIOSim());
                this.wrist = new WristSubsystem(new WristIOSim());
                this.grappler = new GrapplerSubsystem(new GrapplerIOSim());
                break;
            case COMP_BOT: // Main robot for competition
            // Note - Fall through intentional
            default:
                this.swerve = new SwerveSubsystem(
                    new SwerveIOCTRE<BaseTunerConstants>(BaseTunerConstants.class)
                );

                this.vision = new VisionSubsystem(
                    new CameraIOArducam(VisionConstants.CAM_NAME, VisionConstants.CAMERA_OFFSET)
                );

                this.coralIntake = new CoralRollerSubsystem(new CoralRollerIOTalonFX(new PortConfig(31, false)));
                this.algaeIntake = new AlgaeRollerSubsystem(new AlgaeRollerIOTalonFX(new PortConfig(32, true)));
                this.wrist = new WristSubsystem(new WristIOTalonFX(new PortConfig(32, false)));
                this.grappler = new GrapplerSubsystem(new GrapplerIOTalonFX(new PortConfig(33, false)));
                break;
        }

        this.activeSubsystems = new SubsystemList(
            // Add all active subsystems here
            swerve,
            vision,
            coralIntake,
            algaeIntake,
            wrist,
            grappler
        );

        this.activeSubsystems.forEach(s -> {
            s.enableSubsystem(true);
        });
    }

    /**
     * Returns a command that runs when the robot enters a specific match mode
     */
    public Command onModeInitCommand(MatchMode mode) {
        BaseSubsystem<?, ?>[] subs = new BaseSubsystem[activeSubsystems.size()];
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
    public List<BaseSubsystem<?, ?>> getAllSubsystemsAsList() {
        return activeSubsystems;
    }

    /**
     * Returns all active subsystem in the form of an array
     */
    public BaseSubsystem<?, ?>[] getAllSubsystemsAsArray() {
        BaseSubsystem<?, ?>[] subsystems = new BaseSubsystem[activeSubsystems.size()];
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