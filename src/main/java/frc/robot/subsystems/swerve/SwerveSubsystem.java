// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.*;
import frc.robot.subsystems.BaseSubsystem;
import lib.MatchMode;
import lib.Utils;
import lib.control.DriveScaler;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import java.util.function.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.*;
import com.pathplanner.lib.config.RobotConfig;

public class SwerveSubsystem extends BaseSubsystem<SwerveIO, ESwerveData> {
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    private Timer trajectoryTimer = new Timer();

    

    private double translationScaling = 1d;
    private double rotateScaling = 1d;

    private boolean robotRelative;

    private DriveScaler xScaler = new DriveScaler(
        DriveScaler.ScaleType.QUADRATIC, 
        true, 
        0.03
    );

    private DriveScaler yScaler = new DriveScaler(
        DriveScaler.ScaleType.QUADRATIC, 
        true, 
        0.03
    );

    private DriveScaler rotScaler = new DriveScaler(
        DriveScaler.ScaleType.LINEAR, 
        true, 
        0.03
    );

    private HolonomicDriveController pathFollowerCtrl = new HolonomicDriveController(
        PATH_FOLLOW_TRANSLATE_GAINS.toPIDController(),
        PATH_FOLLOW_TRANSLATE_GAINS.toPIDController(),
        PATH_FOLLOW_ROTATE_GAINS.toProfiledPIDController()
    );

    public SwerveSubsystem(SwerveIO io) {
        super(io, ESwerveData.class);

        this.pathFollowerCtrl.setTolerance(new Pose2d(
            new Translation2d(
                PATH_FOLLOW_TRANSLATE_GAINS.toPIDController().getPositionTolerance(), 
                PATH_FOLLOW_TRANSLATE_GAINS.toPIDController().getPositionTolerance()),
            new Rotation2d(PATH_FOLLOW_ROTATE_GAINS.toProfiledPIDController().getPositionTolerance())
        ));

        try {
            AutoBuilder.configure(
                this::getCurrentPosition, 
                (pose) -> resetPoseCallback(pose), 
                this::getWheelSpeeds, 
                (speeds) -> this.drive(speeds),
                new PPHolonomicDriveController(
                    SwerveConstants.PATH_FOLLOW_TRANSLATE_GAINS.toPIDConstants(), 
                    SwerveConstants.PATH_FOLLOW_ROTATE_GAINS.toPIDConstants()
                ),
                RobotConfig.fromGUISettings(),
                Robot.IS_RED_ALLIANCE,
                this
            );
        } catch (Exception e) {
            System.out.println("GUI Settings not properly configured for PathPlanner");
        }

        Pathfinding.setPathfinder(new LocalADStar());
    }

    /**
     * Send a {@code ChassisSpeeds} to the drivetrain, robot-relative
     *
     * @param speeds the speeds to run the drivetrain at
     */
    private void drive(ChassisSpeeds speeds){
        this.desiredSpeeds = speeds;
        io.drive(speeds, false);
    }

    /**
     * Send a {@code ChassisSpeeds} to the drivetrain, robot-relative
     *
     * @param speeds the speeds to run the drivetrain at
     */
    private void drive(ChassisSpeeds speeds, boolean robotRelative){
        this.desiredSpeeds = speeds;
        io.drive(
            speeds,
            robotRelative
        );
    }

    /**
     * Get the current robot yaw as a Rotation2d
     */
    private Rotation2d getYaw() {
        return io.getYaw();
    }

    /**
     * Get the current position of the swerve as judged by odometry.
     */
    private Pose2d getCurrentPosition() {
        return io.getCurrentOdometryPosition();
    }

    /**
     * Get the current translational and rotational speeds of the drivetrain
     */
    private ChassisSpeeds getWheelSpeeds() {
        return io.getWheelSpeeds();
    }

    /**
     * Resets the known pose of the robot to the given pose
     */
    public void resetPoseCallback(Pose2d pose) {
        io.setKnownOdometryPose(pose);
    }

    /**
     * Adds the given pose towards the odometry estimate
     */
    public void addVisionMeasurementCallback(Pose2d pose) {
        io.addVisionMeasurement(pose, Timer.getFPGATimestamp());
    }

    @Override
    public void onModeInit(MatchMode mode) {
        stop();
    }

    @Override
    public void simulationPeriodic() {
        Pose2d pose = new Pose2d(
            getCurrentPosition().getTranslation(),
            getCurrentPosition().getRotation()
        );

        Robot.FIELD.getField().setRobotPose(pose==null?new Pose2d():pose);
    }

    @Override
    public void periodicTelemetry() {
        this.data.map(ESwerveData.CURRENT_POSE, getCurrentPosition());
        this.data.map(ESwerveData.CURRENT_WHEEL_SPEEDS, getWheelSpeeds());
        this.data.map(ESwerveData.CURRENT_YAW, getYaw());
        this.data.map(ESwerveData.DESIRED_SPEEDS, desiredSpeeds);
        this.io.updateInputs();
    }

    /**
     * Stop the swerve (feed zeros for all target velocities)
     */
    @Override
    public void stop(){
        drive(new ChassisSpeeds());
    }

    // ========================================= \\
    // ================ Commands =============== \\
    // ========================================= \\

    /**
     * Set whether human-input-processed joystick input should be slowed
     *
     * @param snailMode whether to slow the drivetrain
     */
    public Command setSnailMode(boolean snailMode){
        return runOnce(() -> {
            this.translationScaling = snailMode ? SNAIL_MODE_TRANSLATIONAL_SCALING : DEFAULT_TRANSLATIONAL_SCALING;
            this.rotateScaling = snailMode ? SNAIL_MODE_ROTATION_SCALING : DEFAULT_ROTATION_SCALING;
        });
    }

    /**
     * Set whether human-input-processed joystick input should be robot-relative
     * (as opposed to field-relative)
     *
     * @param robotRelative whether to run the drivetrain robot-relative
     */
    public Command setRobotRelative(boolean robotRelative){
        return runOnce(() -> this.robotRelative = robotRelative);
    }

    /**
     * Define whatever direction the robot is facing as forward
     */
    public Command resetHeading(){
        return runOnce(() -> io.resetHeading());
    }

    public Command resetPose(Pose2d pose){
        return runOnce(() -> resetPoseCallback(pose));
    }

    public Command resetPose(Pose2d pose, BooleanSupplier flip) {
        return runOnce(() -> {
            Pose2d resetToPose = pose;
            if (flip.getAsBoolean()) {
                resetToPose = new Pose2d(
                    new Translation2d(
                        Robot.FIELD.getFieldLength() - pose.getX(),
                        Robot.FIELD.getFieldWidth() - pose.getY()
                    ),
                    Rotation2d.fromDegrees(180).minus(pose.getRotation())
                );
            }
            resetPoseCallback(resetToPose);
        });
    }

    public Command resetAlliancePose(Pose2d pose) {
        return runOnce(() -> {
            Pose2d resetToPose = pose;
            if (DriverStation.getAlliance().isPresent() && 
                DriverStation.getAlliance().get() == Alliance.Red) {
                resetToPose = new Pose2d(
                    new Translation2d(
                        Robot.FIELD.getFieldLength() - pose.getX(),
                        Robot.FIELD.getFieldWidth() - pose.getY()
                    ),
                    Rotation2d.fromDegrees(180).minus(pose.getRotation())
                );
            }
            resetPoseCallback(resetToPose);
        });
    }

    /**
     * Command to drive the swerve with translation and rotation processed for human input
     * 
     * @param translateX a lambda returning the driver's X input
     * @param translateY a lambda returning the driver's Y input
     * @param rotate a lambda returning the driver's rotate input
     *
     * @return the command
     */
    public Command joystickDrive(DoubleSupplier translateX, DoubleSupplier translateY, DoubleSupplier rotate) {
        return run(() -> {
            double driveTranslateX = xScaler.scale(translateX.getAsDouble());
            double driveTranslateY = yScaler.scale(translateY.getAsDouble());
            double driveRotate = rotScaler.scale(rotate.getAsDouble());

            ChassisSpeeds speeds = new ChassisSpeeds(
                driveTranslateY * translationScaling * MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveTranslateX * translationScaling * MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveRotate * rotateScaling * MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND
            );

            drive(speeds, !robotRelative);
        });
    }

    public Command driveFieldOriented(ChassisSpeeds speeds) {
        return run(() -> {
            drive(speeds, false);
        });
    }

    public Command driveRobotRelative(ChassisSpeeds speeds) {
        return run(() -> {
            drive(speeds, true);
        });
    }

    /**
     * Command to drive the swerve with translation processed for human input and
     * rotation controlled by the snap-to PID controller (snapping to the passed-in)
     * angle
     *
     * @param angle the angle to snap to
     * @param translateX a lambda returning the driver's X input
     * @param translateY a lambda returning the driver's Y input
     *
     * @return the command
     */
    public Command snapToAngle(Rotation2d angle, DoubleSupplier translateX, DoubleSupplier translateY) {
        return joystickDrive(translateX, translateY, () -> {
            double currYaw = Math.toRadians(getYaw().getDegrees()%360);
            double errorAngle = angle.getRadians() - currYaw;

            if(errorAngle > Math.PI){
                errorAngle -= 2*Math.PI;
            }
            else if(errorAngle <= -Math.PI){
                errorAngle += 2*Math.PI;
            }

            return SNAP_TO_PID.calculate(0, errorAngle);
        });
    }

    public Command followTrajectory(Trajectory trajectory) {
        return followTrajectory(trajectory, () -> false);
    }

    public Command followTrajectory(Trajectory trajectory, BooleanSupplier flip) {
        return runOnce(() -> { // Initialize
            trajectoryTimer.reset();
            trajectoryTimer.restart();
            this.pathFollowerCtrl = new HolonomicDriveController(
                PATH_FOLLOW_TRANSLATE_GAINS.toPIDController(),
                PATH_FOLLOW_TRANSLATE_GAINS.toPIDController(),
                PATH_FOLLOW_ROTATE_GAINS.toProfiledPIDController()
            );

            this.pathFollowerCtrl.setTolerance(new Pose2d(
                new Translation2d(
                    PATH_FOLLOW_TRANSLATE_GAINS.toPIDController().getPositionTolerance(), 
                    PATH_FOLLOW_TRANSLATE_GAINS.toPIDController().getPositionTolerance()),
                new Rotation2d(PATH_FOLLOW_ROTATE_GAINS.toProfiledPIDController().getPositionTolerance())
            ));
        })
        .andThen(resetPose(trajectory.getInitialPose(), flip)).onlyIf(() -> Robot.isSimulation()) // Reset pose only if we are in simulation
        .andThen(run(() -> { // Drive along path
            State desiredState = trajectory.sample(trajectoryTimer.get());
            if(flip.getAsBoolean()){
                desiredState = Utils.mirrorState(desiredState, flip.getAsBoolean());
            }

            ChassisSpeeds driveSpeeds = pathFollowerCtrl.calculate(
                getCurrentPosition(),
                desiredState,
                desiredState.poseMeters.getRotation()
            );

            drive(driveSpeeds);

        })).until(() -> // End condition
            trajectoryTimer.hasElapsed(trajectory.getTotalTimeSeconds()) && 
                (pathFollowerCtrl.atReference() || !Robot.isReal())
        )
        .andThen(() -> { // Reset to stop at path completion
            drive(new ChassisSpeeds());
        });
    }

    public Command addVisionMeasurement(Pose2d visionRobotPoseMeters) {
        return runOnce(() -> io.addVisionMeasurement(visionRobotPoseMeters, Timer.getFPGATimestamp()));
    }

    public Command driveToPose(Pose2d targetPose, BooleanSupplier flipPose) {
        return new ConditionalCommand(
            AutoBuilder.pathfindToPoseFlipped(targetPose, DRIVE_TO_POSE_CONSTRAINTS),
            AutoBuilder.pathfindToPose(targetPose, DRIVE_TO_POSE_CONSTRAINTS),
            flipPose
        );
    }

    public Command driveToPose(Pose2d targetPose) {
        return driveToPose(targetPose, () -> false);
    }

    public Command driveToAlliancePose(Pose2d targetPose) {
        return driveToPose(targetPose, Robot.IS_RED_ALLIANCE);
    }
}