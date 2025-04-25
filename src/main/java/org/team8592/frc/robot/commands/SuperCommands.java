package org.team8592.frc.robot.commands;

import org.team8592.frc.robot.Constants;
import org.team8592.frc.robot.Robot;
import org.team8592.frc.robot.subsystems.swerve.SwerveSubsystem;
import org.team8592.frc.robot.subsystems.vision.VisionSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

/**
 * A class that holds all commands that are inter-subsystem or require multiple different parts or mechanisms
 */
public final class SuperCommands {
    public static Command updateOdometryWithVision(SwerveSubsystem swerve, VisionSubsystem vision) {
        return new ConditionalCommand(
            // If we are disabled, reset robot pose to the vision-based estimated pose
            swerve.resetPose(() -> vision.getRobotPoseVision().get().estimatedPose.toPose2d())
                .onlyWhile(() -> Math.abs(vision.getPoseAmbiguityRatio()) < Constants.VISION.MAX_ACCEPTABLE_AMBIGUITY), 

            // If we are enabled, add the vision-based estimated pose to the odometry
            swerve.addVisionMeasurement(() -> vision.getRobotPoseVision().get().estimatedPose.toPose2d())
                .onlyWhile(() -> Math.abs(vision.getPoseAmbiguityRatio()) < Constants.VISION.MAX_ACCEPTABLE_AMBIGUITY),

            // If driver station is disabled
            () -> DriverStation.isDisabled()
        )
        .onlyWhile(() -> vision.getRobotPoseVision().isPresent() && Robot.isReal())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
}