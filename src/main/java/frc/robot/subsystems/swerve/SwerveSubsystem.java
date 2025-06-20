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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.*;
import frc.robot.subsystems.Subsystem;
import lib.MatchMode;
import lib.SmoothingFilter;
import lib.Utils;
import lib.control.DriveScaler;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.RobotConfig;

public class SwerveSubsystem extends Subsystem<SwerveIO, ESwerveData> {
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

    private boolean robotRelative;

    private SmoothingFilter smoothingFilter;


    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    private Timer trajectoryTimer = new Timer();

    private PIDController snapToCtrl = SNAP_TO_GAINS.toPIDController();

    private double translationScaling = 1d;
    private double rotateScaling = 1d;

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

    private ProfiledPIDController driveToPoseXCtrl = DRIVE_TO_POSE_GAINS.toProfiledPIDController();
    private ProfiledPIDController driveToPoseYCtrl = DRIVE_TO_POSE_GAINS.toProfiledPIDController();

    public SwerveSubsystem(SwerveIO io) {
        super(io, ESwerveData.class);
        smoothingFilter = new SmoothingFilter(
            TRANSLATION_SMOOTHING_AMOUNT,
            TRANSLATION_SMOOTHING_AMOUNT,
            ROTATION_SMOOTHING_AMOUNT
        );

        // this.io.registerTelemetry((state) -> {});

        this.pathFollowerCtrl.setTolerance(new Pose2d(
            new Translation2d(
                PATH_FOLLOW_TRANSLATE_GAINS.toPIDController().getPositionTolerance(), 
                PATH_FOLLOW_TRANSLATE_GAINS.toPIDController().getPositionTolerance()),
            new Rotation2d(PATH_FOLLOW_ROTATE_GAINS.toProfiledPIDController().getPositionTolerance())
        ));

        try {
            AutoBuilder.configure(
                this::getCurrentPosition, 
                (pose) -> io.setKnownOdometryPose(pose), 
                this::getWheelSpeeds, 
                (speeds) -> this.drive(speeds),
                new PPHolonomicDriveController(
                    SwerveConstants.PATH_FOLLOW_TRANSLATE_GAINS.toPIDConstants(), 
                    SwerveConstants.PATH_FOLLOW_ROTATE_GAINS.toPIDConstants()
                ),
                RobotConfig.fromGUISettings(),
                Robot.isRedAlliance,
                this
            );
        } catch (Exception e) {
            System.out.println("GUI Settings not properly configured for PathPlanner");
        }
        
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
        double driveTranslateX = xScaler.scale(rawX);
        double driveTranslateY = yScaler.scale(rawY);
        double driveRotate = rotScaler.scale(rawRot);

        ChassisSpeeds currentSpeeds;

        currentSpeeds = smoothingFilter.smooth(new ChassisSpeeds(
            driveTranslateY * translationScaling * MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
            driveTranslateX * translationScaling * MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
            driveRotate * rotateScaling * MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND
        ));

        return currentSpeeds;
    }

    @Override
    public void onModeInit(MatchMode mode) {
        stop();
        driveToPoseXCtrl.reset(0, 0);
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
        io.updateInputs();

        this.db.set(ESwerveData.CURRENT_POSE, getCurrentPosition());
        this.db.set(ESwerveData.CURRENT_WHEEL_SPEEDS, getWheelSpeeds());
        this.db.set(ESwerveData.CURRENT_YAW, getYaw());
        this.db.set(ESwerveData.DESIRED_SPEEDS, desiredSpeeds);
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
        return runOnce(() -> {
            this.translationScaling = slowMode ? TRANSLATE_POWER_SLOW : TRANSLATE_POWER_FAST;
            this.rotateScaling = slowMode ? ROTATE_POWER_SLOW : ROTATE_POWER_FAST;
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
        return runOnce(() -> io.setKnownOdometryPose(pose));
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
            io.setKnownOdometryPose(resetToPose);
        });
    }

    public Command resetPoseFlipOnlyX(Pose2d pose, BooleanSupplier flip) {
        return runOnce(() -> {
            Pose2d resetToPose = pose;
            if (flip.getAsBoolean()) {
                resetToPose = new Pose2d(
                    new Translation2d(
                        Robot.FIELD.getFieldLength() - pose.getX(),
                        pose.getY()
                    ),
                    Rotation2d.fromDegrees(180).minus(pose.getRotation())
                );
            }
            io.setKnownOdometryPose(resetToPose);
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
            io.setKnownOdometryPose(resetToPose);
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

    public Command driveToPose(Pose2d pose) {
        return this.driveToPose(pose, () -> false);
    }

    public Command driveToPose(Pose2d pose, BooleanSupplier flipPose) {
        return this.run(
            () -> {
                logger.log("Error Degrees", snapToCtrl.getPositionError());

                Pose2d desiredPose = Utils.mirrorPose(pose, flipPose.getAsBoolean());

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

                boolean isRedAlliance = DriverStation.getAlliance().isPresent() && 
                DriverStation.getAlliance().get() == Alliance.Red;

                Pose2d desiredPose = Utils.mirrorPose(pose, isRedAlliance);

                this.drive(new ChassisSpeeds(
                    -driveToPoseXCtrl.calculate(getCurrentPosition().getX(), desiredPose.getX()), 
                    -driveToPoseYCtrl.calculate(getCurrentPosition().getY(), desiredPose.getY()), 
                    snapToCtrl.calculate(getYaw().getDegrees(), desiredPose.getRotation().getDegrees())
                ), DriveModes.FIELD_RELATIVE);
            }
        ).until(() -> driveToPoseXCtrl.atSetpoint() && driveToPoseYCtrl.atSetpoint() && snapToCtrl.atSetpoint());
    }

    public Command addVisionMeasurement(Supplier<Pose2d> visionRobotPoseMeters) {
        return runOnce(() -> io.addVisionMeasurement(visionRobotPoseMeters.get(), Timer.getFPGATimestamp()));
    }
}