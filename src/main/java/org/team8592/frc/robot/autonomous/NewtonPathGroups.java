package org.team8592.frc.robot.autonomous;

import org.team8592.lib.Utils;
import org.team8592.lib.pathing.newtonpath.NewtonTrajectory;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public class NewtonPathGroups {
    public static final class TestSimple {
        public static final NewtonTrajectory FIRST_PATH = new NewtonTrajectory(
            new TrajectoryConfig(4.75, 4.75),
            Utils.createPose(1, 2, new Rotation2d()),
            Utils.createPose(3, 3, Rotation2d.fromDegrees(90)),
            Utils.createPose(5, 2, new Rotation2d())
        ).withEndRotation(Rotation2d.fromDegrees(45));
    }
}