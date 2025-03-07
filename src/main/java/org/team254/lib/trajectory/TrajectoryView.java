package org.team254.lib.trajectory;

import org.team254.lib.geometry.State;

public interface TrajectoryView<S extends State<S>> {
    TrajectorySamplePoint<S> sample(final double interpolant);

    double first_interpolant();

    double last_interpolant();

    Trajectory<S> trajectory();
}
