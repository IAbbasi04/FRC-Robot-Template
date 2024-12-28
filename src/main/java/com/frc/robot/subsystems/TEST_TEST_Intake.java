package com.frc.robot.subsystems;

import com.frc.robot.Robot;
import com.lib.team8592.MatchMode;
import com.lib.team8592.PIDProfile;
import com.lib.team8592.hardware.motor.NewtonMotorIO;
import com.lib.team8592.hardware.motor.spark.SparkFlexMotor;

public class TEST_TEST_Intake extends NewtonSubsystem {
    private SparkFlexMotor rollerMotor;
    private NewtonMotorIO<SparkFlexMotor> rollerIO;

    private double desiredRollerRPM = 0d;

    private PIDProfile rollerGains = new PIDProfile()
        .setP(1E-4)
        // .setD(1E-2)
        .setA(0.01)
        .setV(5E-4)
        // .setS(0.1)
        .setMaxVelocity(5000d)
        .setMaxAcceleration(5000d);

    public TEST_TEST_Intake(boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.rollerMotor = new SparkFlexMotor(29);
        this.rollerIO = new NewtonMotorIO<>(rollerGains, 1d, 1d, rollerMotor);
    }

    public void setRollerVelocity(double desiredRPM) {
        if (!isEnabled()) return; // Do nothing if subsystem not in active list

        this.desiredRollerRPM = desiredRPM;
        this.rollerIO.setVelocityRPM(desiredRPM);
    }

    @Override
    public void onInit(MatchMode mode) {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void periodicLogs() {
        this.logger.log("Desired RPM", this.desiredRollerRPM);
        this.logger.log("Current RPM", this.rollerIO.getVelocityRPM());
        this.logger.log("Applied RPM", this.rollerIO.getAppliedVelocityRPM());
    }

    @Override
    public void periodicOutputs() {
        this.rollerIO.update(Robot.CLOCK.dt(), Robot.isSimulation());
    }

    @Override
    public void stop() {
        this.setRollerVelocity(0d);
    }
}