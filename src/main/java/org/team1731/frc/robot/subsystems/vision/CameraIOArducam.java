package org.team1731.frc.robot.subsystems.vision;

import java.util.*;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.*;

public class CameraIOArducam extends CameraIO {
    public CameraIOArducam(String name, Transform3d robotPoseToCameraPose) {
        super(name, robotPoseToCameraPose);
    }

    @Override
    public void updateInputs(Pose3d robotPose) {
        // No necessary updates for real camera
    }

    @Override
    public List<PhotonTrackedTarget> getAllTargets() {
        results = this.camera.getAllUnreadResults();
        if (this.camera.getAllUnreadResults().size() == 0) return new ArrayList<>();
        return this.camera.getAllUnreadResults().get(results.size() - 1).targets;
    }
}