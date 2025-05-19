package lib.team254.trajectory;

import lib.team254.geometry.State;

public interface TrajectoryView<S extends State<S>> {
    TrajectorySamplePoint<S> sample(final double interpolant);

    double first_interpolant();

    double last_interpolant();

    Trajectory<S> trajectory();
}
