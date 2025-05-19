package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public abstract class BaseAuto extends WrapperCommand {
    public BaseAuto(Command... commands) {
        super(new SequentialCommandGroup(commands));
    }

    public BaseAuto(Command command) {
        super(command);
    }

    public static BaseAuto getDefaultAuto() {
        return new BaseAuto() {
            @Override
            public Pose2d getInitialPose() {
                return new Pose2d();
            }
        };
    }

    public abstract Pose2d getInitialPose();
}