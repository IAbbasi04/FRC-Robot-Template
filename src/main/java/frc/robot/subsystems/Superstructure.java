package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure {
    public static final Mechanism2d superstructureMech = new Mechanism2d(2d, 2d);

    public Superstructure() {
        SmartDashboard.putData(superstructureMech);
    }

}