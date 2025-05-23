package frc.robot.subsystems.roller;

import frc.robot.subsystems.io.VelocitySubsystemIO;

public abstract class RollerIO extends VelocitySubsystemIO {
    @Override
    public void halt() {
        setPercentOutput(0d);
    }
}