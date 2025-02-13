package org.team8592.frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team8592.frc.robot.Robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

public class CameraIOSim extends CameraIO {
    private PhotonCameraSim simCamera;
    private VisionSystemSim simVisionSystem;
    private SimCameraProperties simProperties;

    public CameraIOSim(String name, Transform3d robotPoseToCameraPose) {
        super(name, robotPoseToCameraPose);

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

    @Override
    public List<PhotonTrackedTarget> getAllTargets() {
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

    @Override
    public void updateInputs(Pose3d robotPose) {
        this.simVisionSystem.update(robotPose);
    }
}