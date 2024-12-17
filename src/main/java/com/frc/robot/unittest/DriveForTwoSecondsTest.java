package com.frc.robot.unittest;

import com.frc.robot.subsystems.SubsystemManager;
import com.frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveForTwoSecondsTest extends SingleSubsystemTest<SwerveSubsystem> {
    protected DriveForTwoSecondsTest(SubsystemManager manager) {
        super(manager, manager.getSwerve());
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 2d;
    }

    @Override
    public void initTest() {}

    @Override
    public void updateTest() {
        manager.getSwerve().drive(new ChassisSpeeds(2, 0, 0));
    }

    @Override
    public String getTestName() {
        return "Drive for Two Seconds Test";
    }
}