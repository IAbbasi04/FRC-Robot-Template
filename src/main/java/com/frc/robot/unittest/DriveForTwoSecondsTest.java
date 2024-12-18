package com.frc.robot.unittest;

import com.frc.robot.subsystems.SubsystemManager;
import com.frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveForTwoSecondsTest extends SingleSubsystemTest<SwerveSubsystem> {
    protected DriveForTwoSecondsTest(SubsystemManager manager) {
        super(manager, manager.getSwerve());
    }

    @Override
    public void initTest() {}

    @Override
    public void updateTest() {
        manager.getSwerve().drive(new ChassisSpeeds(2, 0, 0));
    }

    @Override
    public String toString() {
        return "Drive Forward for Two Seconds";
    }

    @Override
    public boolean hasFailed() {
        return timer.get() >= 1d && 
            manager.getSwerve().getDesiredSpeeds().equals(new ChassisSpeeds());
    }

    @Override
    public boolean hasSucceeded() {
        return timer.get() >= 2d && 
            manager.getSwerve().getDesiredSpeeds().equals(new ChassisSpeeds(2, 0, 0));
    }

    @Override
    public boolean hasGivenUp() {
        return timer.get() >= 3d;
    }
}