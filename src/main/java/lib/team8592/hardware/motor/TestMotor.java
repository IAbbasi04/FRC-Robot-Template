package lib.team8592.hardware.motor;

import lib.team8592.PIDProfile;
import lib.team8592.logging.SmartLogger;

import edu.wpi.first.wpilibj2.command.*;

public class TestMotor extends SubsystemBase {
    protected NewtonMotor testMotor; // accesible to other test motors
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
        PortConfig config = new PortConfig(deviceID, inverted);
        if (cls.getSimpleName().equals(SparkFlexMotor.class.getSimpleName())) {
            this.testMotor = new SparkFlexMotor(config);
        } else if (cls.getSimpleName().equals(SparkMaxMotor.class.getSimpleName())) {
            this.testMotor = new SparkMaxMotor(config);
        } else if (cls.getSimpleName().equals(TalonFXMotor.class.getSimpleName())) {
            this.testMotor = new TalonFXMotor(config);
        } else if (cls.getSimpleName().equals(TalonFXSMotor.class.getSimpleName())) {
            this.testMotor = new TalonFXSMotor(config);
        }

        this.logger = new SmartLogger("TestMotor/" + cls.getSimpleName() + "[" + deviceID + "]");
        this.logger.log("Class Type", cls.getSimpleName());
    }

    public TestMotor withFollower(TestMotor other, boolean inverted) {
        other.testMotor.setFollowerTo(this.testMotor, inverted);
        return this;
    }

    public TestMotor withFollower(TestMotor other) {
        return this.withFollower(other, false);
    }

    public TestMotor withGains(PIDProfile profile) {
        this.testMotor.withGains(profile);
        return this;
    }

    public double getRotations() {
        return this.testMotor.getRotations();
    }

    public double getVelocityRPM() {
        return this.testMotor.getVelocityRPM();
    }

    public double getAppliedVoltage() {
        return this.testMotor.getAppliedVoltage();
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
