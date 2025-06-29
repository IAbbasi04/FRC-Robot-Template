package frc.robot.subsystems.vision;    

import java.util.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.*;
import org.team254.geometry.Pose2d;

import edu.wpi.first.math.geometry.*;
import frc.robot.Robot;
import lib.MatchMode;
import lib.subsystem.BaseSubsystem;

public class VisionSubsystem extends BaseSubsystem<CameraIO, VisionData> {
    private List<PhotonTrackedTarget> allVisibleTags = new ArrayList<>();
    private PhotonTrackedTarget bestTarget = new PhotonTrackedTarget();

    public VisionSubsystem(CameraIO io){
        super(io, VisionData.class);
    }

    public boolean isAnyTargetVisible() {
        return io.isAnyTargetVisible();
    }

    public List<PhotonTrackedTarget> getTargets() {
        return io.getAllTargets();
    }

    public Optional<EstimatedRobotPose> getRobotPoseVision() {
        return io.getVisionEstimatedPose();
    }

    public double getPoseAmbiguityRatio() {
        return io.getPoseAmbiguityRatio();
    }

    @Override
    public void onModeInit(MatchMode mode) {
        // So far does nothing
    }

    @Override
    public void simulationPeriodic() {
        // So far does nothing
    }

    @Override
    public void periodicTelemetry() {
        io.updateInputs(new Pose3d(Robot.FIELD.getField().getRobotPose()));

        this.logger.log("Target Visible", io.isAnyTargetVisible());

        // Reset both every robot cycle
        this.allVisibleTags = new ArrayList<>();
        this.bestTarget = null;

        if (!io.isAnyTargetVisible()) return; // Do not log if we do not have any visible tag

        this.bestTarget = io.getBestTarget();
        for (PhotonTrackedTarget target : io.getAllTargets()) { // Grab all visible tags
            allVisibleTags.add(target);
        }
        
        if (Robot.isSimulation()) return; // Do not log below if simulation

        // this.logger.logIf("Best Target Yaw", bestTarget.yaw, -1d, isAnyTargetVisible());
        // this.logger.logIf("Best Target Pitch", bestTarget.pitch, -1d, isAnyTargetVisible());
        // this.logger.logIf("Best Target Skew", bestTarget.skew, -1d, isAnyTargetVisible());
        // this.logger.logIf("Best Target Yaw", bestTarget.yaw, -1d, isAnyTargetVisible());
        // this.logger.logIf("Best Target ID", bestTarget.getDetectedObjectClassID(), -1d, isAnyTargetVisible());
        // this.logger.logIf("Best Target X", bestTarget.getBestCameraToTarget().getX(), -1d, isAnyTargetVisible());
        // this.logger.logIf("Best Target Y", bestTarget.getBestCameraToTarget().getY(), -1d, isAnyTargetVisible());
        // this.logger.logIf("Best Target Z", bestTarget.getBestCameraToTarget().getZ(), -1d, isAnyTargetVisible());

        // this.logger.logIf("Closest Tag ID", io.getClosestTagID(), -1, isAnyTargetVisible());

        this.data.map(VisionData.BEST_TARGET_DATA, bestTarget);
        this.data.map(VisionData.IS_ANY_TARGET_VISIBLE, io.isAnyTargetVisible());
        this.data.mapIf(
            VisionData.ESTIMATED_ROBOT_POSE,
            io.getVisionEstimatedPose().get().estimatedPose.toPose2d(),
            new Pose2d(),
            io.getVisionEstimatedPose().isPresent()
        );

        // this.logger.logIf(
        //     "Estimated Pose", 
        //     getRobotPoseVision().get().estimatedPose.toPose2d(),
        //     new Pose2d(),
        //     getRobotPoseVision().isPresent()
        // );
    }

    @Override
    public void stop() {
        // So far does nothing
    }
}