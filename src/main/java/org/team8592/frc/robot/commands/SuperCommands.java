package org.team8592.frc.robot.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.team8592.frc.robot.Constants;
import org.team8592.frc.robot.Robot;
import org.team8592.frc.robot.subsystems.swerve.SwerveSubsystem;
import org.team8592.frc.robot.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

/**
 * A class that holds all commands that are inter-subsystem or require multiple different parts or mechanisms
 */
public final class SuperCommands {
    public static Command updateOdometryWithVision(SwerveSubsystem swerve, VisionSubsystem vision) {
        return vision.run(() -> {
            Optional<EstimatedRobotPose> estimatedRobotPose = vision.getRobotPoseVision();
            if (estimatedRobotPose.isPresent()) {
                Pose2d robotPose = estimatedRobotPose.get().estimatedPose.toPose2d();
                double ambiguity = vision.getPoseAmbiguityRatio();

                if(Math.abs(ambiguity) < Constants.VISION.MAX_ACCEPTABLE_AMBIGUITY) {
                    if (DriverStation.isDisabled()){
                        swerve.resetPose(robotPose);
                    } else {
                        swerve.addVisionMeasurement(robotPose);
                    }
                }
            }
        })
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .onlyIf(() -> Robot.isReal());
    }
}