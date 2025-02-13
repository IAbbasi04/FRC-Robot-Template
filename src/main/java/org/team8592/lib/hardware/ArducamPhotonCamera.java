package org.team8592.lib.hardware;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;

import org.team8592.lib.field.FieldLayout;

public class ArducamPhotonCamera {
    private PhotonCamera camera;
    private PhotonCameraSim simCamera;
    private VisionSystemSim simVisionSystem;
    private SimCameraProperties simProperties;

    private List<PhotonPipelineResult> results = new ArrayList<>();

    private PhotonPoseEstimator estimator;

    private boolean isSimulation = false;

    private Transform3d cameraPoseToRobotPose = new Transform3d();

    public ArducamPhotonCamera(String name, Transform3d robotPoseToCameraPose, AprilTagFieldLayout fieldLayout, boolean isSimulation) {
        this.camera = new PhotonCamera(name);
        this.cameraPoseToRobotPose = robotPoseToCameraPose;

        this.isSimulation = isSimulation;

        this.simProperties = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        this.simProperties.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        this.simProperties.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        this.simProperties.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        this.simProperties.setAvgLatencyMs(35);
        this.simProperties.setLatencyStdDevMs(5);

        this.simCamera = new PhotonCameraSim(camera, simProperties);
        this.simVisionSystem = new VisionSystemSim(name);

        this.simVisionSystem.addAprilTags(fieldLayout);
        this.simVisionSystem.addCamera(simCamera, robotPoseToCameraPose);

        this.estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotPoseToCameraPose);
    }

    public void updateSim(Pose3d robotPose) {
        if (isSimulation) {
            this.simCamera.enableDrawWireframe(true);
            this.simVisionSystem.update(robotPose);
        }
    }

    public VisionSystemSim getSimulation() {
        return this.simVisionSystem;
    }

    public PhotonTrackedTarget getClosestTag() {
        if (isAnyTargetVisible()) {
            List<PhotonTrackedTarget> targets = getAllTargets();
            List<Double> distances = new ArrayList<Double>();

            targets.forEach(
                (target) -> {
                    double x = target.getBestCameraToTarget().getX();
                    double y = target.getBestCameraToTarget().getY();
                    distances.add(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
                }
            );


            return targets.get(distances.indexOf(Collections.min(distances)));
        }
        else {
            return null;
        }
    }

    public Optional<EstimatedRobotPose> getVisionEstimatedPose() {
        return this.estimator.update(results.get(results.size() - 1));
    }

    public double getPoseAmbiguityRatio() {
        return this.getBestTarget().poseAmbiguity;
    }

    public int getClosestTagID() {
        if (getClosestTag() == null) return -1;
        return getClosestTag().getFiducialId();
    }

    /*
     * The photonlib camera object
     */
    public PhotonCamera getCamera() {
        return this.camera;
    }

    /**
     * Grabs all visible targets
     */
    public List<PhotonTrackedTarget> getAllTargets() {
        results = this.camera.getAllUnreadResults();
        if (this.camera.getAllUnreadResults().size() == 0) return new ArrayList<>();
        return this.camera.getAllUnreadResults().get(results.size() - 1).targets;
    }

    /**
     * Grabs the target most matching the current pipeline
     */
    public PhotonTrackedTarget getBestTarget() {
        if (getAllTargets().size() == 0) return null;

        return getAllTargets().get(0);
    }

    /**
     * Sets the current pipeline index
     */
    public void setPipeline(int pipeline) {
        this.camera.setPipelineIndex(pipeline);
    }

    /**
     * Whether there is any target visible in the camera
     */
    public boolean isAnyTargetVisible() {
        return this.getBestTarget() != null;
    }

    /**
     * Whether a particular AprilTag is visible in the camera
     */
    public boolean isAprilTagVisible(int id) {
        for (PhotonTrackedTarget target : getAllTargets()) {
            if (target.getFiducialId() == id) return true;
        }
        return false;
    }

    /**
     * Gets the yaw offset from the best visible target
     */
    public Rotation2d getYawToBestTarget() {
        if (!isAnyTargetVisible()) return null; // Return null if no target visible
        return Rotation2d.fromDegrees(this.getBestTarget().getYaw());
    }

    /**
     * Gets the yaw offset from the best visible target
     */
    public Rotation2d getYawToTarget(PhotonTrackedTarget target) {
        if (!isAnyTargetVisible()) return null; // Return null if no target visible
        return Rotation2d.fromDegrees(target.getYaw());
    }

    /**
     * Gets the yaw offset from the best visible target
     */
    public Rotation2d getYawToAprilTag(int id) {
        for (PhotonTrackedTarget target : getAllTargets()) {
            if (target.getFiducialId() == id) return Rotation2d.fromDegrees(target.getYaw());
        }
        return null; // Return null if the particular april tag is not seen by the camera
    }

    /**
     * The calculated 2d field position of the robot based on april tag vision data
     */
    @Deprecated (forRemoval = false)
    public Pose2d getEstimatedFieldPose(FieldLayout layout) {
        return PhotonUtils.estimateFieldToRobotAprilTag(
            getBestTarget().getBestCameraToTarget(), 
            layout.getAprilTagLayout().getTagPose(
                getBestTarget().getFiducialId()
            ).get(), 
            cameraPoseToRobotPose
        ).toPose2d();
    }
}