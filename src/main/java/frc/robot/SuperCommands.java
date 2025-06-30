package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.*;

public class SuperCommands {
    public static Command updateSwerveTelemetry(VisionSubsystem vision, SwerveSubsystem swerve) {
        return vision.run(
            () -> {
                Optional<EstimatedRobotPose> estimatedRobotPose = vision.data.pull(VisionData.ESTIMATED_ROBOT_POSE);
                if (estimatedRobotPose.isPresent()) {
                    Pose2d robotPose = estimatedRobotPose.get().estimatedPose.toPose2d();
                    double ambiguity = vision.data.pull(VisionData.POSE_AMBIGUITY_RATIO);

                    if(Math.abs(ambiguity) < VisionConstants.MAX_ACCEPTABLE_AMBIGUITY) {
                        if (DriverStation.isDisabled()){
                            swerve.doOnce(swerve.resetPose(robotPose));
                        } else {
                            swerve.doOnce(swerve.addVisionMeasurement(robotPose));
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