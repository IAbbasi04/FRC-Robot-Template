package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.*;

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

                if(Math.abs(ambiguity) < VisionConstants.MAX_ACCEPTABLE_AMBIGUITY) {
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