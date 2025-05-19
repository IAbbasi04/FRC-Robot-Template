// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1731.frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import static org.team1731.frc.robot.Constants.SWERVE.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.team1731.frc.robot.*;
import org.team1731.frc.robot.Constants.SWERVE;
import org.team1731.frc.robot.subsystems.NewtonSubsystem;
import org.team1731.lib.MatchMode;
import org.team1731.lib.SmoothingFilter;
import org.team1731.lib.Utils;

public class SwerveSubsystem extends NewtonSubsystem {
    /**
     * Small enum to control whether to drive robot- or field-
     * relative for {@link SwerveSubsystem#drive(ChassisSpeeds, DriveModes)}
     */
    public enum DriveModes{
        /** Drive robot-relative */
        ROBOT_RELATIVE,
        /** Switch between robot- and field-relative depending on driver input */
        AUTOMATIC,
        /** Drive field-relative */
        FIELD_RELATIVE
    }

    private boolean isSlowMode;
    private boolean robotRelative;

    private SmoothingFilter smoothingFilter;

    private SwerveIO io;

    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    private Timer trajectoryTimer = new Timer();

    private PIDController snapToCtrl = SNAP_TO_GAINS.toPIDController();;

    private HolonomicDriveController pathFollowerCtrl = new HolonomicDriveController(
        SWERVE.PATH_FOLLOW_TRANSLATE_GAINS.toPIDController(),
        SWERVE.PATH_FOLLOW_TRANSLATE_GAINS.toPIDController(),
        SWERVE.PATH_FOLLOW_ROTATE_GAINS.toProfiledPIDController()
    );

    private ProfiledPIDController driveToPoseXCtrl = SWERVE.PATH_FOLLOW_TRANSLATE_GAINS.toProfiledPIDController();
    private ProfiledPIDController driveToPoseYCtrl = SWERVE.PATH_FOLLOW_TRANSLATE_GAINS.toProfiledPIDController();

    public SwerveSubsystem(SwerveIO io) {

        smoothingFilter = new SmoothingFilter(
            TRANSLATION_SMOOTHING_AMOUNT,
            TRANSLATION_SMOOTHING_AMOUNT,
            ROTATION_SMOOTHING_AMOUNT
        );

        this.io = io;
        this.io.registerTelemetry((state) -> {});

        this.pathFollowerCtrl.setTolerance(new Pose2d(
            new Translation2d(
                SWERVE.PATH_FOLLOW_TRANSLATE_GAINS.toPIDController().getPositionTolerance(), 
                SWERVE.PATH_FOLLOW_TRANSLATE_GAINS.toPIDController().getPositionTolerance()),
            new Rotation2d(SWERVE.PATH_FOLLOW_ROTATE_GAINS.toProfiledPIDController().getPositionTolerance())
        ));
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
    private void drive(ChassisSpeeds speeds, DriveModes mode){
        this.desiredSpeeds = speeds;
        io.drive(
            speeds,
            switch(mode){
                case FIELD_RELATIVE:
                    yield true;
                case AUTOMATIC:
                    yield !robotRelative;
                case ROBOT_RELATIVE:
                    yield false;
            }
        );
    }

    /**
     * Get the current robot yaw as a Rotation2d
     */
    public Rotation2d getYaw() {
        return io.getYaw();
    }

    /**
     * Get the current position of the swerve as judged by odometry.
     */
    public Pose2d getCurrentPosition() {
        return io.getCurrentOdometryPosition();
    }

    /**
     * Get the current translational and rotational speeds of the drivetrain
     */
    public ChassisSpeeds getWheelSpeeds() {
        return io.getWheelSpeeds();
    }

    /**
     * Reset the robot's known position.
     *
     * @param pose the pose to set the robot's known position to.
     */
    public void resetPose(Pose2d pose) {
        io.setKnownOdometryPose(pose);
        logger.log("Reset Pose", pose);
    }
    
    public void resetPose(Pose2d pose, boolean flip) {
        if(flip){
            Pose2d flipped = new Pose2d(
                new Translation2d(
                    Robot.FIELD.getFieldLength()-pose.getX(),
                    pose.getY()
                ),
                Rotation2d.fromDegrees(180).minus(pose.getRotation())
            );
            io.setKnownOdometryPose(flipped);
            return;
        }
        io.setKnownOdometryPose(pose);
    }

    /**
     * Process joystick inputs for human control
     *
     * @param rawX the raw X input from a joystick. Should be -1 to 1
     * @param rawY the raw Y input from a joystick. Should be -1 to 1
     * @param rawRot the raw rotation input from a joystick. Should be -1 to 1
     * @param fieldRelativeAllowed if this is true, switch between field- and
     * robot-relative based on {@link SwerveSubsystem#robotRelative}. Otherwise, force
     * robot-relative.
     *
     * @return a ChassisSpeeds ready to be sent to the swerve.
     */
    private ChassisSpeeds processJoystickInputs(double rawX, double rawY, double rawRot){
        double driveTranslateY = (
            rawY >= 0
            ? (Math.pow(Math.abs(rawY), JOYSTICK_EXPONENT))
            : -(Math.pow(Math.abs(rawY), JOYSTICK_EXPONENT))
        );

        double driveTranslateX = (
            rawX >= 0
            ? (Math.pow(Math.abs(rawX), JOYSTICK_EXPONENT))
            : -(Math.pow(Math.abs(rawX), JOYSTICK_EXPONENT))
        );

        double driveRotate = (
            rawRot >= 0
            ? (Math.pow(Math.abs(rawRot), JOYSTICK_EXPONENT))
            : -(Math.pow(Math.abs(rawRot), JOYSTICK_EXPONENT))
        );

        ChassisSpeeds currentSpeeds;

        if (isSlowMode) {
            currentSpeeds = smoothingFilter.smooth(new ChassisSpeeds(
                driveTranslateY * TRANSLATE_POWER_SLOW * MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveTranslateX * TRANSLATE_POWER_SLOW * MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveRotate * ROTATE_POWER_SLOW * MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND
            ));
        }
        else {
            currentSpeeds = smoothingFilter.smooth(new ChassisSpeeds(
                driveTranslateY * TRANSLATE_POWER_FAST * MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveTranslateX * TRANSLATE_POWER_FAST * MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveRotate * ROTATE_POWER_FAST * MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND
            ));
        }

        return currentSpeeds;
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters) {
        io.addVisionMeasurement(visionRobotPoseMeters, Timer.getFPGATimestamp());
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
        logger.log("Current Pose", getCurrentPosition());
        logger.log("Desired Speeds", desiredSpeeds);
        io.updateInputs();
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
     * @param slowMode whether to slow the drivetrain
     */
    public Command setSlowMode(boolean slowMode){
        return runOnce(() -> this.isSlowMode = slowMode);
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
            drive(processJoystickInputs(
                translateX.getAsDouble(),
                translateY.getAsDouble(),
                rotate.getAsDouble()
            ), DriveModes.AUTOMATIC);
        });
    }

    public Command driveFieldOriented(ChassisSpeeds speeds) {
        return run(() -> {
            drive(speeds, DriveModes.FIELD_RELATIVE);
        });
    }

    public Command driveRobotRelative(ChassisSpeeds speeds) {
        return run(() -> {
            drive(speeds, DriveModes.ROBOT_RELATIVE);
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
        return run(() -> {
            double currYaw = Math.toRadians(getYaw().getDegrees()%360);
            double errorAngle = angle.getRadians() - currYaw;

            if(errorAngle > Math.PI){
                errorAngle -= 2*Math.PI;
            }
            else if(errorAngle <= -Math.PI){
                errorAngle += 2*Math.PI;
            }

            double rotSpeed = snapToCtrl.calculate(0, errorAngle);

            drive(
                processJoystickInputs(
                    translateX.getAsDouble(),
                    translateY.getAsDouble(),
                    rotSpeed
                ), DriveModes.AUTOMATIC);
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
                SWERVE.PATH_FOLLOW_TRANSLATE_GAINS.toPIDController(),
                SWERVE.PATH_FOLLOW_TRANSLATE_GAINS.toPIDController(),
                SWERVE.PATH_FOLLOW_ROTATE_GAINS.toProfiledPIDController()
            );

            this.pathFollowerCtrl.setTolerance(new Pose2d(
                new Translation2d(
                    SWERVE.PATH_FOLLOW_TRANSLATE_GAINS.toPIDController().getPositionTolerance(), 
                    SWERVE.PATH_FOLLOW_TRANSLATE_GAINS.toPIDController().getPositionTolerance()),
                new Rotation2d(SWERVE.PATH_FOLLOW_ROTATE_GAINS.toProfiledPIDController().getPositionTolerance())
            ));

            if (Robot.isSimulation()) { resetPose(trajectory.getInitialPose(), flip.getAsBoolean()); }
        }).andThen(run(() -> { // Drive along path
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

    public Command driveToPose(Pose2d pose) {
        return this.driveToPose(pose, () -> false);
    }

    public Command driveToPose(Pose2d pose, BooleanSupplier flipPose) {
        return this.run(
            () -> {
                logger.log("Error Degrees", snapToCtrl.getPositionError());

                Pose2d desiredPose = pose;
                if (flipPose.getAsBoolean()) {
                    desiredPose = Utils.mirrorPose(pose, flipPose.getAsBoolean());
                }

                this.drive(new ChassisSpeeds(
                    -driveToPoseXCtrl.calculate(getCurrentPosition().getX(), desiredPose.getX()), 
                    -driveToPoseYCtrl.calculate(getCurrentPosition().getY(), desiredPose.getY()), 
                    snapToCtrl.calculate(getYaw().getDegrees(), desiredPose.getRotation().getDegrees())
                ), DriveModes.FIELD_RELATIVE);
            }
        ).until(() -> driveToPoseXCtrl.atSetpoint() && driveToPoseYCtrl.atSetpoint() && snapToCtrl.atSetpoint());
    }

    public Command driveToAlliancePose(Pose2d pose) {
        return this.run(
            () -> {
                logger.log("Error Degrees", snapToCtrl.getPositionError());

                Pose2d desiredPose = pose;
                if (Suppliers.IS_RED_ALLIANCE.getAsBoolean()) {
                    desiredPose = Utils.mirrorPose(pose, Suppliers.IS_RED_ALLIANCE.getAsBoolean());
                }

                this.drive(new ChassisSpeeds(
                    -driveToPoseXCtrl.calculate(getCurrentPosition().getX(), desiredPose.getX()), 
                    -driveToPoseYCtrl.calculate(getCurrentPosition().getY(), desiredPose.getY()), 
                    snapToCtrl.calculate(getYaw().getDegrees(), desiredPose.getRotation().getDegrees())
                ), DriveModes.FIELD_RELATIVE);
            }
        ).until(() -> driveToPoseXCtrl.atSetpoint() && driveToPoseYCtrl.atSetpoint() && snapToCtrl.atSetpoint());
    }
}