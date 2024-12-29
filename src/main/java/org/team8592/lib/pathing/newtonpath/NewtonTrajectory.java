package org.team8592.lib.pathing.newtonpath;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.*;

public class NewtonTrajectory {
    private ArrayList<Pose2d> poses = new ArrayList<>();
    private Rotation2d endRotation = new Rotation2d();
    private Trajectory trajectory = new Trajectory();

    public NewtonTrajectory(TrajectoryConfig config, Pose2d... poses) {
        if (poses.length < 2) {
            throw new UnsupportedOperationException("Need at least 2 posees in path!!");
        }

        this.trajectory = generateWPILibTrajectory(config, poses[0], poses[1]);
        this.poses.add(poses[0]);
        this.poses.add(poses[1]);

        for (int i = 2; i < poses.length - 1; i++) {
            this.poses.add(poses[i]);
            this.trajectory = this.trajectory.concatenate(
                generateWPILibTrajectory(
                    config, 
                    poses[i], 
                    poses[i + 1]
                )
            );
        }

        this.trajectory = generateWPILibTrajectory(config, poses);
    }

    public static Trajectory generateWPILibTrajectory(TrajectoryConfig config, Pose2d... poses) {
        List<Translation2d> interiorPoses = new ArrayList<>();
        for (int i = 1; i < poses.length - 1; i++) {
            interiorPoses.add(poses[i].getTranslation());
        }

        return TrajectoryGenerator.generateTrajectory(
            poses[0], 
            interiorPoses, 
            poses[poses.length - 1], 
            config
        );
    }

    public NewtonTrajectory withEndRotation(Rotation2d endRot) {
        this.endRotation = endRot;
        return this;
    }

    public Trajectory getTrajectory() {
        return this.trajectory;
    }

    public Rotation2d getEndRotation() {
        return this.endRotation;
    }

    public Pose2d getStartPose() {
        return this.poses.get(0);
    }
}
