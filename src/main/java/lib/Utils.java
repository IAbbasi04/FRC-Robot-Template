package lib;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.Robot;

/**
 * General utility class with helper methods
 */
public final class Utils {
    /**
     * Clamps the value between a given range
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(Math.min(value, max), min);
    }

    /**
     * Clamps the value between a given range
     */
    public static double clamp(double value, double range) {
        return Math.max(Math.min(value, range), -range);
    }

    /**
     * Rounds the number to a certain number of decimal places
     */
    public static double roundTo(double value, double decimalPlaces) {
        return Math.round(value*Math.pow(10, decimalPlaces))/Math.pow(10, decimalPlaces);
    }

    /**
     * Whether the particlar value is within a specified tolerance of the target
     */
    public static boolean isWithin(double value, double target, double tolerance) {
        return Math.abs(target - value) <= tolerance;
    }

    /**
     * Mirrors a Pose2d from one side of the field to the corresponding spot on the other side
     */
    public static Pose2d mirrorPose(Pose2d pose, boolean flip) {
        Pose2d newPose = pose;
        if (flip) {
            newPose = new Pose2d(
                Robot.FIELD.getFieldLength() - pose.getX(),
                Robot.FIELD.getFieldWidth() - pose.getY(),
                Rotation2d.fromRadians(Math.PI).minus(pose.getRotation())
            );
        }

        return newPose;
    }

    /**
     * Mirrors a Trjectory.State from one side of the field to the corresponding spot on the other side
     */
    public static State mirrorState(State state, boolean flip) {
        return new State(
            state.timeSeconds, 
            state.velocityMetersPerSecond, 
            state.accelerationMetersPerSecondSq, 
            mirrorPose(state.poseMeters, flip),
            state.curvatureRadPerMeter
        );
    }

    // Calculates the moment of inertia for a roller
    public static double getMOIForRoller(double massKG, double radiusMeters) {
        return (massKG * Math.pow(radiusMeters, 2d)) / 2d;
    }

    /**
     * Returns what the motor voltage would be after simulated friction
     */
    public static double simulateMotorFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }

    /**
     * The initial pose for the given Choreo path
     */
    public static Pose2d getStartPoseFromTrajectory(String file) {
        return Choreo.loadTrajectory(file).get().getInitialPose(
            DriverStation.getAlliance().isPresent() && 
            DriverStation.getAlliance().get() == Alliance.Red
        ).get();
    }

    /**
     * The WPILib trajectory based on a given Choreo path
     */
    @SuppressWarnings("unchecked")
    public static Trajectory getTrajectoryFromChoreo(String file) {
        List<SwerveSample> choreoSamples = ((choreo.trajectory.Trajectory<SwerveSample>) Choreo.loadTrajectory(file).get()).samples();
        ArrayList<State> states = new ArrayList<>();

        for (SwerveSample sample : choreoSamples) {
            double velocity = Math.hypot(sample.vx, sample.vy);
            double acceleration = Math.hypot(sample.ax, sample.ay);

            states.add(
                new State(
                    sample.t,
                    velocity,
                    acceleration,
                    sample.getPose(),
                    sample.omega / velocity
                )
            );
        }

        return new Trajectory(states);
    }

    /**
     * The WPILib trajectory based on a PathPlanner path
     */
    public static Trajectory getTrajectoryFromPathPlanner(String file) {
        ArrayList<State> states = new ArrayList<>();

        try {
            PathPlannerTrajectory trajectory = PathPlannerPath.fromPathFile(file).getIdealTrajectory(RobotConfig.fromGUISettings()).get();

            double lastTime = 0d;
            double lastVel = 0d;
            double lastHeading = 0d;

            for (PathPlannerTrajectoryState state : trajectory.getStates()) {
                double dT = state.timeSeconds - lastTime;
                double dV = state.linearVelocity - lastVel;
                double dTheta = (state.heading.getRadians() - lastHeading) / dT;

                states.add(
                    new State(
                        state.timeSeconds,
                        state.linearVelocity,
                        dV / dT,
                        state.pose,
                        dTheta / dT
                    )
                );

                lastTime = state.timeSeconds;
                lastVel = state.linearVelocity;
                lastHeading = state.heading.getRadians();
            }
        } catch (Exception e) {
            System.out.println("PathPlanner Path Does Not Exist: " + e.getMessage());
        }

        return new Trajectory(states);
    }
}