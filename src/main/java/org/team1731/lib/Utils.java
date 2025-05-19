package org.team1731.lib;

import org.team1731.frc.robot.Robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class Utils {
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

    public static Pose2d mirrorPose(Pose2d pose, boolean flip) {
        Pose2d newPose = pose;
        if (flip) {
            newPose = new Pose2d(
                Robot.FIELD.getFieldLength() - pose.getX(),
                /*Robot.FIELD.getFieldWidth() - */pose.getY(),
                Rotation2d.fromRadians(Math.PI).minus(pose.getRotation())
            );
        }

        return newPose;
    }

    public static State mirrorState(State state, boolean flip) {
        return new State(
            state.timeSeconds, 
            state.velocityMetersPerSecond, 
            state.accelerationMetersPerSecondSq, 
            mirrorPose(state.poseMeters, flip),
            state.curvatureRadPerMeter
        );
    }

    public static double getMOIForRoller(double massKG, double radiusMeters) {
        return (massKG * Math.pow(radiusMeters, 2d)) / 2d;
    }
}