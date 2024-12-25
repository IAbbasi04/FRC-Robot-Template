package com.frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class RobotProfile {
    private Mechanism2d mechanism;
    private MechanismRoot2d mechRoot;

    public RobotProfile(double mechWidth, double mechHeight) {
        this.mechanism = new Mechanism2d(mechWidth, mechHeight);
    }

    public RobotProfile withMechanismBase(String rootName, double rootX, double rootY) {
        this.mechRoot = this.mechanism.getRoot(rootName, rootX, rootY);
        return this;
    }

    public RobotProfile append(MechanismLigament2d ligament) {
        this.mechRoot.append(ligament);
        return this;
    }
}