package frc.robot.subsystems.vision;    

import java.util.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.*;

import edu.wpi.first.math.geometry.*;

import frc.robot.Robot;

import lib.MatchMode;
import lib.subsystem.BaseSubsystem;

public class VisionSubsystem extends BaseSubsystem {
    private List<PhotonTrackedTarget> allVisibleTags = new ArrayList<>();

    public VisionSubsystem(CameraIO io){
    }

    @Override
    public void onModeInit(MatchMode mode) {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void periodicTelemetry() {
        io.updateInputs(new Pose3d(Robot.FIELD.getField().getRobotPose()));

        this.logger.log("Target Visible", io.isAnyTargetVisible());

        // Reset every robot cycle
        this.allVisibleTags = new ArrayList<>();

        if (!io.isAnyTargetVisible()) return; // Do not log if we do not have any visible tag

        for (PhotonTrackedTarget target : io.getAllTargets()) { // Grab all visible tags
            allVisibleTags.add(target);
        }
        
        if (Robot.isSimulation()) return; // Do not log below if simulation

        this.data.map(VisionData.BEST_TARGET_DATA, io.getBestTarget());
        this.data.map(VisionData.IS_ANY_TARGET_VISIBLE, io.isAnyTargetVisible());
        this.data.map(VisionData.POSE_AMBIGUITY_RATIO, io.getPoseAmbiguityRatio());
        this.data.mapIf(
            VisionData.ESTIMATED_ROBOT_POSE,
            io.getVisionEstimatedPose(),
            Optional.of(new EstimatedRobotPose(new Pose3d(), 0d, new ArrayList<>(), PoseStrategy.LOWEST_AMBIGUITY)),
            io.getVisionEstimatedPose().isPresent()
        );
    }

    @Override
    public void stop() {
        // So far does nothing
    }
}