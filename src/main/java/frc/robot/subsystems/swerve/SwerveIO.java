package frc.robot.subsystems.swerve;

import java.util.function.Consumer;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.io.SubsystemIO;

public abstract class SwerveIO implements SubsystemIO {
    protected ChassisSpeeds targetSpeeds = new ChassisSpeeds();

    public abstract void drive(ChassisSpeeds speeds, boolean driveFieldRelative);

    public abstract void resetHeading();

    public abstract Rotation2d getYaw();

    public abstract Pose2d getCurrentOdometryPosition();

    public abstract void setKnownOdometryPose(Pose2d currentPose);

    public abstract void updateInputs();

    public abstract void brake();

    public abstract void pointAt(Rotation2d direction);

    public abstract void snapToHeading(ChassisSpeeds targetSpeeds, Rotation2d heading);

    public abstract void registerTelemetry(Consumer<SwerveDriveState> driveState);

    public abstract void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds);

    public abstract ChassisSpeeds getWheelSpeeds();

    public ChassisSpeeds getTargetSpeeds() {
        return this.targetSpeeds;
    }
}