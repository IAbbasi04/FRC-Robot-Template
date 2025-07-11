package org.team254.trajectory;

import org.team254.geometry.State;

public class TrajectoryPoint<S extends State<S>> {
    protected final S state_;
    protected final int index_;

    public TrajectoryPoint(final S state, int index) {
        state_ = state;
        index_ = index;
    }

    public TrajectoryPoint(TrajectoryPoint<S> point, int index) {
        state_ = point.state();
        index_ = index;
    }

    public S state() {
        return state_;
    }

    public int index() {
        return index_;
    }
}
