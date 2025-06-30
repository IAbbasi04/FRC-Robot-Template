package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lib.subsystem.io.ISubsystemIO;

public abstract class SwerveIO implements ISubsystemIO {
    public abstract void drive(ChassisSpeeds speeds, boolean driveFieldRelative);

    public abstract void resetHeading();

    /**
     * Get the current robot yaw as a Rotation2d
     */
    public abstract Rotation2d getYaw();

    /**
     * Get the current position of the swerve as judged by odometry.
     */
    public abstract Pose2d getCurrentOdometryPosition();

    public abstract void setKnownOdometryPose(Pose2d currentPose);

    public abstract void updateInputs();

    public abstract void brake();

    public abstract void pointAt(Rotation2d direction);

    public abstract void snapToHeading(ChassisSpeeds targetSpeeds, Rotation2d heading);

    public abstract void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds);

    /**
     * Get the current translational and rotational speeds of the drivetrain
     */
    public abstract ChassisSpeeds getWheelSpeeds();
}