package com.frc.robot.subsystems;

import com.frc.robot.Robot;
import com.lib.team8592.MatchMode;
import com.lib.team8592.PIDProfile;
import com.lib.team8592.Utils;
import com.lib.team8592.hardware.SimIO;
import com.lib.team8592.hardware.motor.NewtonMotorIO;
import com.lib.team8592.hardware.motor.spark.SparkFlexMotor;

public class TEST_TEST_Intake extends NewtonSubsystem {
    private SparkFlexMotor rollerMotor;
    private NewtonMotorIO<SparkFlexMotor> rollerIO;
    private SimIO simIO;

    private double desiredRollerRPM = 0d;

    private PIDProfile rollerGains = new PIDProfile()
        .setP(2000d)
        .setMaxVelocity(5000d)
        .setMaxAcceleration(5000d);

    public TEST_TEST_Intake(boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.rollerMotor = new SparkFlexMotor(29);
        this.rollerMotor.withGains(rollerGains);

        this.rollerIO = new NewtonMotorIO<>(rollerGains, 1d, 1d, rollerMotor);
        this.simIO = new SimIO(
            0.5, 
            Utils.getMOIForRoller(2d, 1d), 
            rollerMotor
        );
    }

    public void setRollerVelocity(double desiredRPM) {
        if (!isEnabled()) return; // Do nothing if subsystem not in active list

        this.desiredRollerRPM = desiredRPM;
        this.rollerIO.setVelocityRPM(desiredRPM);
        this.simIO.setVelocityRPM(desiredRPM);
    }

    @Override
    public void onInit(MatchMode mode) {}

    @Override
    public void simulationPeriodic() {
        this.simIO.update(Robot.CLOCK.dt());
    }

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