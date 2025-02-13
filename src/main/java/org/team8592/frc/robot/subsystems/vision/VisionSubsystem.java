package org.team8592.frc.robot.subsystems.vision;    

import java.util.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.*;

import edu.wpi.first.math.geometry.*;
import org.team8592.frc.robot.subsystems.NewtonSubsystem;
import org.team8592.lib.MatchMode;
import org.team8592.frc.robot.Robot;

public class VisionSubsystem extends NewtonSubsystem<VisionCommands> {
    private CameraIO cameraIO;

    private List<PhotonTrackedTarget> allVisibleTags = new ArrayList<>();
    private PhotonTrackedTarget bestTarget = new PhotonTrackedTarget();

    public VisionSubsystem(CameraIO cameraIO, boolean logToShuffleboard){
        super(logToShuffleboard);
        super.commands = new VisionCommands(this);
        this.cameraIO = cameraIO;
    }

    public boolean isAnyTargetVisible() {
        return cameraIO.isAnyTargetVisible();
    }

    public List<PhotonTrackedTarget> getTargets() {
        return cameraIO.getAllTargets();
    }

    public Optional<EstimatedRobotPose> getRobotPoseVision() {
        return cameraIO.getVisionEstimatedPose();
    }

    public double getPoseAmbiguityRatio() {
        return cameraIO.getPoseAmbiguityRatio();
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
        cameraIO.updateInputs(new Pose3d(Robot.FIELD.getField().getRobotPose()));

        this.logger.log("Target Visible", cameraIO.isAnyTargetVisible());

        // Reset both every robot cycle
        this.allVisibleTags = new ArrayList<>();
        this.bestTarget = null;

        if (!cameraIO.isAnyTargetVisible()) return; // Do not log if we do not have any visible tag

        this.bestTarget = cameraIO.getBestTarget();
        for (PhotonTrackedTarget target : cameraIO.getAllTargets()) { // Grab all visible tags
            allVisibleTags.add(target);
        }
        
        this.logger.logIf("Best Target Yaw", bestTarget.yaw, -1d, isAnyTargetVisible());
        this.logger.logIf("Best Target Pitch", bestTarget.pitch, -1d, isAnyTargetVisible());
        this.logger.logIf("Best Target Skew", bestTarget.skew, -1d, isAnyTargetVisible());
        this.logger.logIf("Best Target Yaw", bestTarget.yaw, -1d, isAnyTargetVisible());
        this.logger.logIf("Best Target ID", bestTarget.getDetectedObjectClassID(), -1d, isAnyTargetVisible());
        this.logger.logIf("Best Target X", bestTarget.getBestCameraToTarget().getX(), -1d, isAnyTargetVisible());
        this.logger.logIf("Best Target Y", bestTarget.getBestCameraToTarget().getY(), -1d, isAnyTargetVisible());
        this.logger.logIf("Best Target Z", bestTarget.getBestCameraToTarget().getZ(), -1d, isAnyTargetVisible());

        this.logger.logIf("Closest Tag ID", cameraIO.getClosestTagID(), -1, isAnyTargetVisible());

        this.logger.logIf(
            "Estimated Pose", 
            getRobotPoseVision().get().estimatedPose.toPose2d(),
            new Pose2d(),
            getRobotPoseVision().isPresent()
        );
    }

    @Override
    public void stop() {
        // So far does nothing
    }
}