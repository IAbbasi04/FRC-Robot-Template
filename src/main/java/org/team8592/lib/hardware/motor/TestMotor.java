package org.team8592.lib.hardware.motor;

import org.team8592.lib.PIDProfile;
import org.team8592.lib.hardware.motor.spark.*;
import org.team8592.lib.hardware.motor.talonfx.*;
import org.team8592.lib.logging.SmartLogger;

import edu.wpi.first.wpilibj2.command.*;

public class TestMotor extends SubsystemBase {
    private NewtonMotor testMotor;
    private double desiredOutput = 0d;
    private SmartLogger logger;
    private OutputType outputType = OutputType.kNone;

    private enum OutputType {
        kPercentOutput,
        kVelocity,
        kPosition,
        kNone
    }

    public TestMotor(int deviceID, boolean inverted, Class<? extends NewtonMotor> cls) {
        if (cls.getSimpleName().equals(SparkFlexMotor.class.getSimpleName())) {
            this.testMotor = new SparkFlexMotor(deviceID, inverted);
        } else if (cls.getSimpleName().equals(SparkMaxMotor.class.getSimpleName())) {
            this.testMotor = new SparkMaxMotor(deviceID, inverted);
        } else if (cls.getSimpleName().equals(KrakenX60Motor.class.getSimpleName())) {
            this.testMotor = new KrakenX60Motor(deviceID, inverted);
        } else if (cls.getSimpleName().equals(KrakenX60FOCMotor.class.getSimpleName())) {
            this.testMotor = new KrakenX60FOCMotor(deviceID, inverted);
        } else if (cls.getSimpleName().equals(Falcon500Motor.class.getSimpleName())) {
            this.testMotor = new Falcon500Motor(deviceID, inverted);
        } else if (cls.getSimpleName().equals(Falcon500FOCMotor.class.getSimpleName())) {
            this.testMotor = new Falcon500FOCMotor(deviceID, inverted);
        } else if (cls.getSimpleName().equals(MinionMotor.class.getSimpleName())) {
            this.testMotor = new MinionMotor(deviceID, inverted);
        }

        this.logger = new SmartLogger("TestMotor/" + cls.getSimpleName() + "[" + deviceID + "]");
        this.logger.log("Class Type", cls.getSimpleName());
    }

    public TestMotor withGains(PIDProfile profile) {
        this.testMotor.withGains(profile);
        return this;
    }

    public Command setPercentOutput(double percent) {
        return new InstantCommand(() -> {
            this.desiredOutput = percent;
            this.testMotor.setPercentOutput(percent);
            this.outputType = OutputType.kPercentOutput;
        });
    }

    public Command setVelocityRPM(double velocityRPM) {
        return new InstantCommand(() -> {
            this.desiredOutput = velocityRPM;
            this.testMotor.setVelocity(velocityRPM);
            this.outputType = OutputType.kVelocity;
        });
    }

    public Command setPosition(double position) {
        return new InstantCommand(() -> {
            this.desiredOutput = position;
            this.testMotor.setPosition(position);
            this.outputType = OutputType.kPosition;
        });
    }

    @Override
    public void periodic() {
        this.logger.log("Desired Output", desiredOutput);
        this.logger.log("Output Type", outputType);
    }
}
