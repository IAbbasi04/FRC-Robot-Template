package org.team1731.frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.team1731.frc.robot.subsystems.SubsystemManager;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.*;

public abstract class AutoCommand extends WrapperCommand {
    private Command command; // The compiled command of what to run in auto
    
    protected static SubsystemManager manager; // A manager of all active subsystems on the robot
    public static void addSubsystems(SubsystemManager manager){
        AutoCommand.manager = manager;
    }

    public static AutoCommand none() {
        return new AutoCommand() {
            @Override
            public Pose2d getStartPose() {
                return new Pose2d();
            }
        };
    }

    public boolean isDefault() {
        return this.equals(none());
    }

    /**
     * Startup time saving if/when multiple copies of the same path are requested
     */
    private static HashMap<String, Trajectory> cachedChoreoTrajectories = new HashMap<String,Trajectory>();

    protected AutoCommand(Command command) {
        super(command);
        this.command = command;
    }

    protected AutoCommand(Command... commands) {
        super(Commands.sequence(commands));
        this.command = Commands.sequence(commands);
    }

    public AutoCommand() {
        super(Commands.none());
        this.command = Commands.none();
    }

    public abstract Pose2d getStartPose();

    public Command getAutoRoutine() {
        return command;
    }

    @SuppressWarnings("unchecked")
    public static Trajectory getChoreoTrajectory(String name) {
        if(cachedChoreoTrajectories.containsKey(name)){
            return cachedChoreoTrajectories.get(name);
        }
        else{
            try{
               Trajectory wpilibTrajectory = fromChoreoPath((choreo.trajectory.Trajectory<SwerveSample>) Choreo.loadTrajectory(name).get());

                cachedChoreoTrajectories.put(name, wpilibTrajectory);
                return wpilibTrajectory;
            }
            catch(Exception e){
                throw new RuntimeException(e);
            }
        }
    }

    protected static Pose2d getStartPoseFromChoreoTrajectory(String name) {
        if(!cachedChoreoTrajectories.containsKey(name)){
            getChoreoTrajectory(name); // Adds the path to the cached trajectory map
        }
        return cachedChoreoTrajectories.get(name).getInitialPose();
    }

    /**
     * Convert a Chreo path into a WPILib trajectory
     *
     * @param path the Choreo to convert
     * @return the path converted to a WPILib trajectory
     */
    protected static Trajectory fromChoreoPath(choreo.trajectory.Trajectory<SwerveSample> path){
        List<SwerveSample> choreoSamples = path.samples();
        ArrayList<State> wpilibStates = new ArrayList<>();

        // Convert all the Choreo states to WPILib trajectory states and add
        // them to the wpilibStates ArrayList
        for (SwerveSample choreoSample : choreoSamples) {
            double metersPerSecond = Math.sqrt(Math.pow(choreoSample.vx, 2)+Math.pow(choreoSample.vy, 2));

            State wpilibState = new State(
                choreoSample.t,
                metersPerSecond,
                Math.sqrt(Math.pow(choreoSample.ax, 2)+Math.pow(choreoSample.ay, 2)),
                choreoSample.getPose(),
                choreoSample.omega/metersPerSecond
            );
            wpilibStates.add(wpilibState);
        }
        return new Trajectory(wpilibStates);
    }
}