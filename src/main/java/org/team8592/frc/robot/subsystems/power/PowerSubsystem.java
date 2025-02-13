package org.team8592.frc.robot.subsystems.power;

import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.team8592.frc.robot.Robot;
import org.team8592.frc.robot.subsystems.NewtonSubsystem;
import org.team8592.frc.robot.subsystems.SubsystemCommands;
import org.team8592.lib.MatchMode;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class PowerSubsystem extends NewtonSubsystem<SubsystemCommands<?>>{
    public PowerSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);

        if (Robot.isReal()) {
            LoggedPowerDistribution.getInstance(1, ModuleType.kRev);
        }
    }

    @Override
    public void onModeInit(MatchMode mode) {
        logger.log("Current Power Mode", mode.toString());
    }

    @Override
    public void periodicTelemetry() {

    }

    @Override
    public void stop() {}
}