package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.*;
import lib.MatchMode;
import lib.Utils;
import lib.subsystem.BaseSubsystem;

import java.util.function.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.*;
import com.pathplanner.lib.config.RobotConfig;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class SwerveSubsystem extends BaseSubsystem<SwerveIO, SwerveData> {
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    private Timer trajectoryTimer = new Timer();

    private double translationScaling = 1d;
    private double rotateScaling = 1d;

    private boolean robotRelative;

    private HolonomicDriveController pathFollowerCtrl = new HolonomicDriveController(
        PATH_FOLLOW_TRANSLATE_GAINS.toPIDController(),
        PATH_FOLLOW_TRANSLATE_GAINS.toPIDController(),
        PATH_FOLLOW_ROTATE_GAINS.toProfiledPIDController()
    );

    public SwerveSubsystem(SwerveIO io) {
        super(io, SwerveData.class);

        this.pathFollowerCtrl.setTolerance(new Pose2d(
            new Translation2d(
                PATH_FOLLOW_TRANSLATE_GAINS.toPIDController().getPositionTolerance(), 
                PATH_FOLLOW_TRANSLATE_GAINS.toPIDController().getPositionTolerance()),
            new Rotation2d(PATH_FOLLOW_ROTATE_GAINS.toProfiledPIDController().getPositionTolerance())
        ));

        try {
            AutoBuilder.configure(
                io::getCurrentOdometryPosition,
                (pose) -> io.setKnownOdometryPose(pose),
                io::getWheelSpeeds,
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
        this.drive(speeds, false);
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

    @Override
    public void onModeInit(MatchMode mode) {
        stop();
    }

    @Override
    public void simulationPeriodic() {
        Pose2d pose = new Pose2d(
            io.getCurrentOdometryPosition().getTranslation(),
            io.getCurrentOdometryPosition().getRotation()
        );

        Robot.FIELD.getField().setRobotPose(pose==null?new Pose2d():pose);
    }

    @Override
    public void periodicTelemetry() {
        this.data.map(SwerveData.CURRENT_POSE, io.getCurrentOdometryPosition());
        this.data.map(SwerveData.CURRENT_WHEEL_SPEEDS, io.getWheelSpeeds());
        this.data.map(SwerveData.CURRENT_YAW, io.getYaw());
        this.data.map(SwerveData.DESIRED_SPEEDS, desiredSpeeds);
        this.io.updateInputs();
    }

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
        return super.createSubsystemCommand("Set Snail Mode", runOnce(() -> {
            this.translationScaling = snailMode ? SNAIL_MODE_TRANSLATIONAL_SCALING : DEFAULT_TRANSLATIONAL_SCALING;
            this.rotateScaling = snailMode ? SNAIL_MODE_ROTATION_SCALING : DEFAULT_ROTATION_SCALING;
        }));
    }

    /**
     * Set whether human-input-processed joystick input should be robot-relative
     * (as opposed to field-relative)
     *
     * @param robotRelative whether to run the drivetrain robot-relative
     */
    public Command setRobotRelative(boolean robotRelative){
        return super.createSubsystemCommand("Set To " + (robotRelative?"Robot":"Field") + " Relative", runOnce(() -> this.robotRelative = robotRelative));
    }

    /**
     * Define whatever direction the robot is facing as forward
     */
    public Command resetHeading(){
        return super.createSubsystemCommand("Reset Heading", runOnce(() -> io.resetHeading()));
    }

    public Command resetPose(Pose2d pose){
        return super.createSubsystemCommand("Reset Pose" + pose.toString(), runOnce(() -> io.setKnownOdometryPose(pose)));
    }

    public Command resetPose(Pose2d pose, BooleanSupplier flip) {
        return super.createSubsystemCommand("Reset Pose" + pose.toString(), runOnce(() -> {
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
        }));
    }

    public Command resetAlliancePose(Pose2d pose) {
        return runOnce(() -> {
            Pose2d resetToPose = pose;
            if (Robot.IS_RED_ALLIANCE.getAsBoolean()) {
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
            double driveTranslateX = X_SCALING.scale(translateX.getAsDouble());
            double driveTranslateY = Y_SCALING.scale(translateY.getAsDouble());
            double driveRotate = ROT_SCALING.scale(rotate.getAsDouble());

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
            double currYaw = Math.toRadians(io.getYaw().getDegrees()%360);
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
                io.getCurrentOdometryPosition(),
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