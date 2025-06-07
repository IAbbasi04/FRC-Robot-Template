package lib.pathing;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.*;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class PathUtils {
    /**
     * The initial pose for the given Choreo path
     */
    public static Pose2d getStartPoseFromTrajectory(String file) {
        return Choreo.loadTrajectory(file).get().getInitialPose(
            DriverStation.getAlliance().isPresent() && 
            DriverStation.getAlliance().get() == Alliance.Red
        ).get();
    }

    /**
     * The WPILib trajectory based on a given Choreo path
     */
    @SuppressWarnings("unchecked")
    public static Trajectory getTrajectoryFromChoreo(String file) {
        List<SwerveSample> choreoSamples = ((choreo.trajectory.Trajectory<SwerveSample>) Choreo.loadTrajectory(file).get()).samples();
        ArrayList<State> states = new ArrayList<>();

        for (SwerveSample sample : choreoSamples) {
            double velocity = Math.hypot(sample.vx, sample.vy);
            double acceleration = Math.hypot(sample.ax, sample.ay);

            states.add(
                new State(
                    sample.t,
                    velocity,
                    acceleration,
                    sample.getPose(),
                    sample.omega / velocity
                )
            );
        }

        return new Trajectory(states);
    }

    public static Trajectory getTrajectoryFromPathPlanner(String file) {
        ArrayList<State> states = new ArrayList<>();

        try {
            PathPlannerTrajectory trajectory = PathPlannerPath.fromPathFile(file).getIdealTrajectory(RobotConfig.fromGUISettings()).get();

            double lastTime = 0d;
            double lastVel = 0d;
            double lastHeading = 0d;

            for (PathPlannerTrajectoryState state : trajectory.getStates()) {
                double dT = state.timeSeconds - lastTime;
                double dV = state.linearVelocity - lastVel;
                double dTheta = (state.heading.getRadians() - lastHeading) / dT;

                states.add(
                    new State(
                        state.timeSeconds,
                        state.linearVelocity,
                        dV / dT,
                        state.pose,
                        dTheta / dT
                    )
                );

                lastTime = state.timeSeconds;
                lastVel = state.linearVelocity;
                lastHeading = state.heading.getRadians();
            }
        } catch (Exception e) {
            System.out.println("PathPlanner Path Does Not Exist: " + e.getMessage());
        }

        return new Trajectory(states);
    }

    public static PathPlannerAuto getPathPlannerAuto(String file) {
        return new PathPlannerAuto(file);
    }
}
