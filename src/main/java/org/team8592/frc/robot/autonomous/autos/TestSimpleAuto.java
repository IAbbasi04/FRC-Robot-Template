package org.team8592.frc.robot.autonomous.autos;

import org.team8592.frc.robot.autonomous.AutoCommand;
import org.team8592.frc.robot.autonomous.NewtonPathGroups;
import org.team8592.frc.robot.commands.largecommands.NewtonFollowerCommand;

import edu.wpi.first.math.geometry.*;

public class TestSimpleAuto extends AutoCommand {
    public TestSimpleAuto() {
        super(
            new NewtonFollowerCommand(
                manager.getSwerve(), 
                NewtonPathGroups.TestSimple.FIRST_PATH
            )
        );
    }

    @Override
    public Pose2d getStartPose() {
        return NewtonPathGroups.TestSimple.FIRST_PATH.getStartPose();
    }
}