package frc.robot.subsystems.roller;

import lib.team1731.io.VelocitySubsystemIO;

public abstract class RollerIO extends VelocitySubsystemIO {
    @Override
    public void halt() {
        setPercentOutput(0d);
    }
}