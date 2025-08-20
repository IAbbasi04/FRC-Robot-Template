package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.ctre.*;

public class CTRESwerve<E extends BaseTunerConstants> {
    private double MaxSpeed = E.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldRelative = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric robotRelative = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
       
    public final CommandSwerveDrivetrain drivetrain = E.createDrivetrain();

    public void drive(ChassisSpeeds speeds, boolean driveFieldRelative) {
        if (driveFieldRelative) {
            drivetrain.setControl(
                fieldRelative.withVelocityX(speeds.vxMetersPerSecond) 
                    .withVelocityY(speeds.vyMetersPerSecond) 
                    .withRotationalRate(speeds.omegaRadiansPerSecond) 
            );
        } else {
            drivetrain.setControl(
                robotRelative
                    .withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond)
            );
        }
    }

    public void resetHeading() {
        drivetrain.seedFieldCentric();
    }

    public Rotation2d getYaw() {
        return drivetrain.getState().Pose.getRotation();
    };

    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    public void setPose(Pose2d currentPose) {        
        drivetrain.resetPose(currentPose);
    }

    public void brake() {
        drivetrain.setControl(brake);
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        drivetrain.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    public void updateInputs() {
        drivetrain.periodic();
    }

    public void snapToHeading(ChassisSpeeds targetSpeeds, Rotation2d heading) {
        this.drive(
            new ChassisSpeeds(
                targetSpeeds.vxMetersPerSecond,
                targetSpeeds.vyMetersPerSecond,
                SNAP_TO_PID.calculate(getYaw().getDegrees(), heading.getDegrees())
            ),
            true
        );
    }

    public ChassisSpeeds getWheelSpeeds() {
        return this.drivetrain.getKinematics().toChassisSpeeds(this.drivetrain.getState().ModuleStates);
    }
}