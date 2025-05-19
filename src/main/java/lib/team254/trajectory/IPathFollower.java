package lib.team254.trajectory;

import lib.team254.geometry.Pose2d;
import lib.team254.geometry.Twist2d;

public interface IPathFollower {
    Twist2d steer(Pose2d current_pose);

    boolean isDone();
}
