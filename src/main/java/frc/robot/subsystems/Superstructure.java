package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionConstants;

/**
 * Super class meant to represent the entire robot; Mainly used for commands used across subsystems
 */
public class Superstructure {
    private SubsystemManager manager;
    public Superstructure(SubsystemManager manager) {
        this.manager = manager;
    }

    public Command updateSwerveTelemetry() {
        return manager.vision.run(
            () -> {
                Optional<EstimatedRobotPose> estimatedRobotPose = manager.vision.getEstimatedRobotPose();
                if (estimatedRobotPose.isPresent()) {
                    Pose2d robotPose = estimatedRobotPose.get().estimatedPose.toPose2d();
                    double ambiguity = manager.vision.getPoseAmbiguity();

                    if(Math.abs(ambiguity) < VisionConstants.MAX_ACCEPTABLE_AMBIGUITY) {
                        if (DriverStation.isDisabled()){
                            manager.swerve.doOnce(manager.swerve.resetPose(robotPose));
                        } else {
                            manager.swerve.doOnce(manager.swerve.addVisionMeasurement(robotPose));
                        }
                    }
                }
            }
        )
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .onlyIf(() -> Robot.isReal())
        .ignoringDisable(true);
    }
}