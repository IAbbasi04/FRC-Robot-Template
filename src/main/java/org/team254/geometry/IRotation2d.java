package org.team254.geometry;

public interface IRotation2d<S> extends State<S> {
    Rotation2d getRotation();

    S rotateBy(Rotation2d other);

    S mirror();
}
