package frc.robot.subsystems.roller;

public class RollerIOSim extends RollerIO {
    public RollerIOSim() {}

    @Override
    public void setVelocity(double desiredRPM) {}

    @Override
    public double getVelocityRPM() {
        return 0d;
    }

    @Override
    public void setPercentOutput(double desiredPercent) {}

    @Override
    public double getOutputVoltage() {
        return 0d;
    }
}