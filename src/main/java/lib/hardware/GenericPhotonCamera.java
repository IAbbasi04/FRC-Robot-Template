package lib.hardware;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.*;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.*;
import org.photonvision.targeting.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class GenericPhotonCamera {
    protected PhotonCamera camera;
    protected String name;
    protected Transform3d robotPoseToCameraPose;

    protected List<PhotonPipelineResult> results = new ArrayList<>();

    protected PhotonPoseEstimator estimator;

    private PhotonCameraSim simCamera;
    private VisionSystemSim simVisionSystem;
    private SimCameraProperties simProperties;

    public GenericPhotonCamera(String name, Transform3d robotPoseToCameraPose) {
        this.name = name;
        this.camera = new PhotonCamera(name);
        this.robotPoseToCameraPose = robotPoseToCameraPose;
        this.estimator = new PhotonPoseEstimator(
            Robot.FIELD.getAprilTagLayout(), 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            robotPoseToCameraPose
        );

        if (Robot.isSimulation()) {
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

            this.simVisionSystem.addAprilTags(Robot.FIELD.getAprilTagLayout());
            this.simVisionSystem.addCamera(simCamera, robotPoseToCameraPose);
        }
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

    public double getPoseAmbiguity() {
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

    public void updatePeriodic() {
        if (Robot.isSimulation()) this.simVisionSystem.update(Robot.FIELD.getSimulatedRobotPose());
    }

    public List<PhotonTrackedTarget> getAllTargets() {
        if (Robot.isSimulation()) {
            List<PhotonTrackedTarget> targets = new ArrayList<>();
            var simTargets = this.simVisionSystem.getVisionTargets();
            List<Integer> ids = new ArrayList<>();
            simTargets.forEach(target -> {
                if (simCamera.canSeeTargetPose(simVisionSystem.getRobotPose(), target)) {
                    Logger.recordOutput("Tags/Tag Pose " + target.fiducialID, Timer.getFPGATimestamp());
                    targets.add(new PhotonTrackedTarget(
                        PhotonUtils.getYawToPose(simVisionSystem.getRobotPose().toPose2d(), target.getPose().toPose2d()).getDegrees(),
                        0d, 
                        0d, 
                        0d, 
                        target.fiducialID, 
                        target.fiducialID, 
                        1f, 
                        new Transform3d(
                            new Translation3d(
                                target.getPose().getX() - simVisionSystem.getRobotPose().getX(), 
                                target.getPose().getY() - simVisionSystem.getRobotPose().getY(), 
                                target.getPose().getZ() - simVisionSystem.getRobotPose().getZ()
                            ), 
                            new Rotation3d(
                                PhotonUtils.getYawToPose(simVisionSystem.getRobotPose().toPose2d(), target.getPose().toPose2d())
                            )
                        ), 
                        new Transform3d(), 
                        0d, 
                        new ArrayList<>(), 
                        new ArrayList<>()
                    ));

                    ids.add(target.fiducialID);
                } else {
                    Logger.recordOutput("Tags/Tag Pose " + target.fiducialID, -1d);
                }
            });

            Logger.recordOutput("Tags/Tags", ids.toString());
            
            return targets;
        }

        results = this.camera.getAllUnreadResults();
        if (this.camera.getAllUnreadResults().size() == 0) return new ArrayList<>();
        return this.camera.getAllUnreadResults().get(results.size() - 1).targets;
    }
}
