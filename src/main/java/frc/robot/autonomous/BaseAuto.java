package frc.robot.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;

public abstract class BaseAuto extends Command {
    protected Command autoCommand = null;
    public boolean isPathPlannerAuto = false;

    public BaseAuto() {}
    
    public BaseAuto(Command... commands) {
        this.autoCommand = new SequentialCommandGroup(commands);
    }

    public BaseAuto(Command command) {
        this.autoCommand = command;
    }

    public static BaseAuto getDefaultAuto() {
        return new BaseAuto() {
            @Override
            public Pose2d getInitialPose() {
                return new Pose2d();
            }
        };
    }

    public static BaseAuto fromPathPlannerAuto(PathPlannerAuto ppAuto) {
        return new BaseAuto(ppAuto) {
            @Override
            public Pose2d getInitialPose() {
                super.isPathPlannerAuto = true;
                return ppAuto.getStartingPose();
            }

            @Override
            public String getName() {
                super.isPathPlannerAuto = true;
                return ppAuto.getName();
            }
        };
    }

    @Override
    public void initialize() {
        if (autoCommand != null) {
            autoCommand.initialize();
        }
    }

    @Override
    public void execute() {
        if (autoCommand != null) {
            autoCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (autoCommand != null) {
            autoCommand.end(interrupted);
        }
    }

    public boolean isFinished() {
        return autoCommand == null || autoCommand.isFinished();
    }

    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    public abstract Pose2d getInitialPose();
}