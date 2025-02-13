package org.team8592.frc.robot.subsystems.vision;    

import java.util.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.*;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.team8592.frc.robot.subsystems.NewtonSubsystem;
import org.team8592.lib.MatchMode;
import org.team8592.lib.hardware.NewtonPhotonCamera;
import org.team8592.frc.robot.Robot;

public class VisionSubsystem extends NewtonSubsystem<VisionCommands> {
    private NewtonPhotonCamera camera;
 
    private PIDController xController = new PIDController(VisionConstants.X_KP, VisionConstants.X_KI, VisionConstants.X_KD);
    private PIDController yController = new PIDController(VisionConstants.Y_KP, VisionConstants.Y_KI, VisionConstants.Y_KD);
    private PIDController rotController = new PIDController(VisionConstants.ROT_KP, VisionConstants.ROT_KI, VisionConstants.ROT_KD);

    private List<PhotonTrackedTarget> allVisibleTags = new ArrayList<>();
    private PhotonTrackedTarget bestTarget = new PhotonTrackedTarget();

    public VisionSubsystem(boolean logToShuffleboard){
        super(logToShuffleboard);

        super.commands = new VisionCommands(this);

        this.camera = new NewtonPhotonCamera(
            VisionConstants.CAM_NAME, 
            VisionConstants.CAMERA_OFFSET, 
            AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
            Robot.isSimulation()
        );
    }

    public boolean isAnyTargetVisible() {
        return camera.isAnyTargetVisible();
    }

    public ChassisSpeeds driveToReef(double offset) {
        PhotonTrackedTarget tag = camera.getBestTarget();
        if (tag == null) {
            return new ChassisSpeeds();
        }

        Transform3d cameraToTarget = tag.getBestCameraToTarget();

        double x = xController.calculate(cameraToTarget.getX(), offset);
        double y = yController.calculate(cameraToTarget.getY(), 0);
        double rot = rotController.calculate(cameraToTarget.getRotation().getAngle(), 0);
        return new ChassisSpeeds(x, y, rot);
    }

    public List<PhotonTrackedTarget> getTargets() {
        return camera.getTargets();
    }

    public Optional<EstimatedRobotPose> getRobotPoseVision() {
        return camera.getVisionEstimatedPose();
    }

    public double getPoseAmbiguityRatio() {
        return camera.getPoseAmbiguityRatio();
    }

    @Override
    public void onModeInit(MatchMode mode) {
        // So far does nothing
    }

    @Override
    public void simulationPeriodic() {
        camera.updateSim(new Pose3d(Robot.FIELD.getField().getRobotPose()));
    }

    @Override
    public void periodicTelemetry() {
        this.logger.log("Target Visible", camera.isAnyTargetVisible());

        // Reset both every robot cycle
        this.allVisibleTags = new ArrayList<>();
        this.bestTarget = null;

        if (!camera.isAnyTargetVisible()) return; // Do not log if we do not have any visible tag

        this.bestTarget = camera.getBestTarget();
        for (PhotonTrackedTarget target : camera.getTargets()) { // Grab all visible tags
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

        this.logger.logIf("Closest Tag ID", camera.getClosestTagID(), -1, isAnyTargetVisible());

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