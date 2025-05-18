package org.team8592.frc.robot.commands;

import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.team8592.frc.robot.commands.proxies.NamedCommand;
import org.team8592.frc.robot.subsystems.swerve.SwerveSubsystem;
import org.team8592.frc.robot.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

/**
 * A class that holds all commands that are inter-subsystem or require multiple different parts or mechanisms
 */
public final class SuperCommands {
    public static Command updateOdometryWithVision(SwerveSubsystem swerve, VisionSubsystem vision) {
        return new NamedCommand("Field Localize", new DeferredCommand(() -> {
            if (RobotBase.isReal()){
                Optional<EstimatedRobotPose> robotPose = vision.getRobotPoseVision();
                if (robotPose.isPresent()) {
                    if(
                        vision.getTargets().size() > 1 || (
                            Math.abs(vision.getPoseAmbiguityRatio()) < 0.1
                            && vision.getTargets().size() > 0 && vision.getTargets().get(0).bestCameraToTarget.getX() < 1.0
                        )
                    ) {
                        if (DriverStation.isDisabled() && !robotPose.get().estimatedPose.toPose2d().equals(new Pose2d())){
                            return swerve.resetPose(() -> robotPose.get().estimatedPose.toPose2d());
                        } else {
                            return swerve.addVisionMeasurement(() -> robotPose.get().estimatedPose.toPose2d(), () -> robotPose.get().timestampSeconds);
                        }
                    }
                }
            }
            return Commands.none();
        }, Set.of(vision)))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .ignoringDisable(true);
    }
}