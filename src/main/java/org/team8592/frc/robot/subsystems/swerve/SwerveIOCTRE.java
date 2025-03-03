package org.team8592.frc.robot.subsystems.swerve;

import java.util.function.Consumer;

import org.team8592.lib.hardware.swerve.CTRESwerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveIOCTRE extends SwerveIO {
    private CTRESwerve drivetrain;

    public SwerveIOCTRE() {
        drivetrain = new CTRESwerve();
    }

    @Override
    public void drive(ChassisSpeeds speeds, boolean driveFieldRelative) {
        drivetrain.drive(speeds, driveFieldRelative);
    }

    @Override
    public void resetHeading() {
        drivetrain.resetHeading();
    }

    @Override
    public Rotation2d getYaw() {
        return drivetrain.getYaw();
    }

    @Override
    public Pose2d getCurrentOdometryPosition() {
        return drivetrain.getCurrentOdometryPosition();
    }

    @Override
    public void setKnownOdometryPose(Pose2d currentPose) {
        drivetrain.setKnownOdometryPose(currentPose);
    }

    @Override
    public void updateInputs() {
        drivetrain.periodic();
    }

    @Override
    public void brake() {
        drivetrain.brake();
    }

    @Override
    public void pointAt(Rotation2d direction) {
        drivetrain.pointAt(direction);
    }

    @Override
    public void registerTelemetry(Consumer<SwerveDriveState> driveState) {
        drivetrain.registerTelemetry(driveState);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        drivetrain.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }
}
