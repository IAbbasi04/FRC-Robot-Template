package com.frc.robot.subsystems;

import com.frc.robot.Robot;
import com.lib.team8592.MatchMode;
import com.lib.team8592.PIDProfile;
import com.lib.team8592.hardware.NewtonMotorIO;
import com.lib.team8592.hardware.motor.spark.SparkFlexMotor;

public class TEST_TEST_Intake extends NewtonSubsystem {
    private SparkFlexMotor rollerMotor;
    private NewtonMotorIO<SparkFlexMotor> rollerIO;
    private PIDProfile rollerGains = new PIDProfile()
        .setP(1d)
        .setMaxVelocity(5000d)
        .setMaxAcceleration(5000d);

    public TEST_TEST_Intake(boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.rollerMotor = new SparkFlexMotor(29);
        this.rollerIO = new NewtonMotorIO<>(rollerGains, 1d, 1d, rollerMotor);
    }

    @Override
    public void onInit(MatchMode mode) {}

    @Override
    public void simulationPeriodic() {
        this.rollerIO.updateSim(Robot.CLOCK.dt());
    }

    @Override
    public void periodicLogs() {}

    @Override
    public void stop() {}
}