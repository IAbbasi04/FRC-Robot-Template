package org.team254.geometry;

public interface ICurvature<S> extends State<S> {
    double getCurvature();

    double getDCurvatureDs();
}
