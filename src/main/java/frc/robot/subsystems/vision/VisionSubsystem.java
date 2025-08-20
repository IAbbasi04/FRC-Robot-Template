package frc.robot.subsystems.vision;    

import lib.MatchMode;
import lib.hardware.GenericPhotonCamera;
import lib.subsystem.BaseSubsystem;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

public class VisionSubsystem extends BaseSubsystem {
    private GenericPhotonCamera camera;

    public VisionSubsystem(){
        camera = new GenericPhotonCamera(CAM_NAME, CAMERA_OFFSET);
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        return camera.getVisionEstimatedPose();
    }

    public double getPoseAmbiguity() {
        return camera.getPoseAmbiguity();
    }

    @Override
    public void onModeInit(MatchMode mode) {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void periodicTelemetry() {
        camera.updatePeriodic();
    }

    @Override
    public void stop() {
        // So far does nothing
    }
}