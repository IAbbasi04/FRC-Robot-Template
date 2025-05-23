package frc.robot.subsystems.wrist;

public class WristIOSim extends WristIO {
    public WristIOSim() {

    }

    @Override
    public void setDegrees(double degrees) {
        // SET SIMULATED WRIST ANGLE
    }

    @Override
    public double getDegrees() {
        return 0d;
    }

    @Override
    public void updateInputs() {
        // CONSISTENTLY UPDATE SIMULATED WRIST ANGLE
    }
}