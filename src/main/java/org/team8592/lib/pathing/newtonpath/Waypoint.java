package org.team8592.lib.pathing.newtonpath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Waypoint {
    public Pose2d pose = new Pose2d();
    public ChassisSpeeds speeds = new ChassisSpeeds();

    public Waypoint(double p_x, double p_y, Rotation2d p_rot, double p_vx, double p_vy, double p_omega) {
        this(new Pose2d(
                new Translation2d(p_x, p_y), p_rot
            ), new ChassisSpeeds(p_vx, p_vy, p_omega)
        );
    }

    public Waypoint(Pose2d p_pose, ChassisSpeeds p_speed) {
        this.pose = p_pose;
        this.speeds = p_speed;
    }

    public Waypoint(Pose2d p_pose) {
        this(p_pose, new ChassisSpeeds());
    }

    public Waypoint() {
        this(new Pose2d(), new ChassisSpeeds());
    }
}