package org.team8592.lib.pathing.newtonpath;

import java.util.List;

import edu.wpi.first.math.trajectory.*;

public class NewtonPath {
    protected List<Trajectory> trajectories;
    protected List<Waypoint> waypoints;
    protected Trajectory overallTrajectory = new Trajectory();

    public NewtonPath(List<Waypoint> waypoints, TrajectoryConfig config) {
        this.waypoints = waypoints;

        if (waypoints.size() < 2) {
            throw new UnsupportedOperationException("Need at least 2 waypoints in path!!");
        }

        this.overallTrajectory = TrajectoryGenerator.generateTrajectory(
            List.of(
                waypoints.get(0).pose,
                waypoints.get(1).pose
            ), config);

        for (int i = 0; i < waypoints.size() - 1; i++) {
            Waypoint current = waypoints.get(i);
            Waypoint next = waypoints.get(i+1);

            trajectories.add(TrajectoryGenerator.generateTrajectory(
                List.of(
                    current.pose,
                    next.pose
                ), config)
            );
        }
    }
}