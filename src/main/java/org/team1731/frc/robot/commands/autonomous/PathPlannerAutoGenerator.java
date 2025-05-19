package org.team1731.frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;

public class PathPlannerAutoGenerator extends AutoCommand {
    public PathPlannerAutoGenerator() {
        super(
            
        );
    }

    @Override
    public Pose2d getStartPose() {
        return new Pose2d();
    }
}