package org.team254.lib.trajectory;

import org.team254.lib.geometry.Pose2d;
import org.team254.lib.geometry.Twist2d;

public interface IPathFollower {
    Twist2d steer(Pose2d current_pose);

    boolean isDone();
}
