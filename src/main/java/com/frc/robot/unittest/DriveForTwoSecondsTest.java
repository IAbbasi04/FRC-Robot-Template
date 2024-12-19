package com.frc.robot.unittest;

import com.frc.robot.subsystems.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveForTwoSecondsTest extends SingleSubsystemTest<SwerveSubsystem> {
    private ChassisSpeeds appliedSpeeds = new ChassisSpeeds();

    protected DriveForTwoSecondsTest(SubsystemManager manager) {
        super(manager, manager.getSwerve());
    }

    @Override
    public Command createTest() {
        appliedSpeeds = new ChassisSpeeds(2d, 0d, 0d);
        return manager.getSwerve().run(() -> {
            manager.getSwerve().drive(appliedSpeeds);
        });
    }

    @Override
    public String toString() {
        return "Drive Forward for Two Seconds";
    }

    @Override
    public boolean hasFailed() {
        return timer.get() >= 1d && manager.getSwerve().getCurrentSpeeds().equals(new ChassisSpeeds());
    }

    @Override
    public boolean hasSucceeded() {
        SmartDashboard.putString("AAA MMM", appliedSpeeds.toString());
        SmartDashboard.putString("BBB MMM", manager.getSwerve().getCurrentSpeeds().toString());

        return timer.get() >= 2d && manager.getSwerve().getCurrentSpeeds().equals(appliedSpeeds);
    }

    @Override
    public boolean hasGivenUp() {
        return timer.get() >= 3d;
    }
}