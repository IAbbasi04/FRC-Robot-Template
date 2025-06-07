package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;

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

    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    public abstract Pose2d getInitialPose();
}