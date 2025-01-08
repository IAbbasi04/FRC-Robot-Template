package org.team8592.frc.robot.subsystems;

import org.team8592.frc.robot.Robot;
import org.team8592.lib.MatchMode;
import org.team8592.lib.hardware.motor.spark.SparkFlexMotor;
import org.team8592.lib.simulation.NewtonRollerSim;

public class IntakeSubsystem extends NewtonSubsystem {
    private SparkFlexMotor rollerMotor;
    private NewtonRollerSim rollerSim;

    protected IntakeSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.rollerMotor = new SparkFlexMotor(29);
    }

    @Override
    public void onInit(MatchMode mode) {}

    @Override
    public void simulationPeriodic() {
        this.rollerSim.update(Robot.CLOCK.dt());
    }

    @Override
    public void periodicLogs() {}

    @Override
    public void stop() {}
    
}
