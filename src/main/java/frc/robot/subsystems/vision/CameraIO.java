package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;
import lib.subsystem.io.ISubsystemIO;

public abstract class CameraIO implements ISubsystemIO {
    protected PhotonCamera camera;
    protected String name;
    protected Transform3d robotPoseToCameraPose;

    protected List<PhotonPipelineResult> results = new ArrayList<>();

    protected PhotonPoseEstimator estimator;

    public CameraIO(String name, Transform3d robotPoseToCameraPose) {
        this.name = name;
        this.camera = new PhotonCamera(name);
        this.robotPoseToCameraPose = robotPoseToCameraPose;
        this.estimator = new PhotonPoseEstimator(
            Robot.FIELD.getAprilTagLayout(), 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            robotPoseToCameraPose
        );
    }

    public boolean isAnyTargetVisible() {
        return this.getBestTarget() != null;
    }

    public boolean isAprilTagVisible(int id) {
        for (PhotonTrackedTarget target : getAllTargets()) {
            if (target.getFiducialId() == id) return true;
        }
        return false;
    }

    public double getPoseAmbiguityRatio() {
        return this.getBestTarget().poseAmbiguity;
    }

    public Optional<EstimatedRobotPose> getVisionEstimatedPose() {
        if (results.size() <= 0) return Optional.empty();
        return this.estimator.update(results.get(results.size() - 1));
    }
    
    public PhotonTrackedTarget getBestTarget() {
        if (getAllTargets().size() == 0) return null;
        return getAllTargets().get(0);
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
    
    public int getClosestTagID() {
        if (getClosestTag() == null) return -1;
        return getClosestTag().getFiducialId();
    }

    public abstract List<PhotonTrackedTarget> getAllTargets();

    public abstract void updateInputs(Pose3d robotPose);
}