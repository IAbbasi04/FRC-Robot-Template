package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.*;

public class CameraIOOculus extends CameraIO {
    private NetworkTable table;

    public CameraIOOculus(String name, Transform3d robotPoseToCameraPose) {
        super(name, robotPoseToCameraPose);
        this.table = NetworkTableInstance.getDefault().getTable(name);
    }

    @Override
    public List<PhotonTrackedTarget> getAllTargets() {
        return new ArrayList<>();
    }

    @Override
    public void updateInputs(Pose3d robotPose) {
    }
    
}