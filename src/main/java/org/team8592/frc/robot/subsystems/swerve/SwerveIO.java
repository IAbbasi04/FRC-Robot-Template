package org.team8592.frc.robot.subsystems.swerve;

import java.util.function.Consumer;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class SwerveIO {
    public abstract void drive(ChassisSpeeds speeds, boolean driveFieldRelative);

    public abstract void resetHeading();

    public abstract Rotation2d getYaw();

    public abstract Pose2d getCurrentOdometryPosition();

    public abstract void setKnownOdometryPose(Pose2d currentPose);

    public abstract void periodic();

    public abstract void brake();

    public abstract void pointAt(Rotation2d direction);

    public abstract void registerTelemetry(Consumer<SwerveDriveState> driveState);

    public abstract void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds);
}