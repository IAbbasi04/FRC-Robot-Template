package lib.hardware.motor;

import lib.PIDProfile;
import lib.logging.SmartLogger;

import edu.wpi.first.wpilibj2.command.*;

/**
 * Test class that treats a motor as a subsystem; useful for easy initial testing of motors and prototypes
 */
public class TestMotor extends SubsystemBase {
    protected BaseMotor testMotor; // accesible to other test motors
    private double desiredOutput = 0d;
    private SmartLogger logger;
    private OutputType outputType = OutputType.kNone;

    private enum OutputType {
        kPercentOutput,
        kVelocity,
        kPosition,
        kNone
    }

    public TestMotor(int deviceID, boolean inverted, Class<? extends BaseMotor> cls) {
        PortConfig config = new PortConfig(deviceID, inverted);
        // if (cls.getSimpleName().equals(SparkFlexMotor.class.getSimpleName())) {
        //     this.testMotor = new SparkFlexMotor(config);
        // } else if (cls.getSimpleName().equals(SparkMaxMotor.class.getSimpleName())) {
        //     this.testMotor = new SparkMaxMotor(config);
        // } else if (cls.getSimpleName().equals(TalonFXMotor.class.getSimpleName())) {
        //     this.testMotor = new TalonFXMotor(config);
        // } else if (cls.getSimpleName().equals(TalonFXSMotor.class.getSimpleName())) {
        //     this.testMotor = new TalonFXSMotor(config);
        // }
        try {
            this.testMotor = cls.getDeclaredConstructor(PortConfig.class).newInstance(config);
        } catch (Exception e) {
            System.out.println("Could not create motor of type: " + cls.getSimpleName());
        }

        this.logger = new SmartLogger("TestMotor/" + cls.getSimpleName() + "[" + deviceID + "]");
        this.logger.log("Class Type", cls.getSimpleName());
    }

    /**
     * Adds an inverted follower motor to this test motor
     */
    public TestMotor withFollower(TestMotor other, boolean inverted) {
        other.testMotor.setFollowerTo(this.testMotor, inverted);
        return this;
    }

    /**
     * Adds a follower motor to this test motor
     */
    public TestMotor withFollower(TestMotor other) {
        return this.withFollower(other, false);
    }

    /**
     * Adds PID gains to this test motor
     */
    public TestMotor withGains(PIDProfile profile) {
        this.testMotor.withGains(profile);
        return this;
    }

    /**
     * Get current position in rotations
     */
    public double getRotations() {
        return this.testMotor.getRotations();
    }

    /**
     * Get current velocity in RPM
     */
    public double getVelocityRPM() {
        return this.testMotor.getVelocityRPM();
    }

    /**
     * Get the voltage applied to the motor
     */
    public double getAppliedVoltage() {
        return this.testMotor.getAppliedVoltage();
    }

    /**
     * Set the desired output of the motor to a percentage of maximum output
     */
    public Command setPercentOutput(double percent) {
        return new InstantCommand(() -> {
            this.desiredOutput = percent;
            this.testMotor.setPercentOutput(percent);
            this.outputType = OutputType.kPercentOutput;
        });
    }

    /**
     * Set the velocity of the motor in RPM
     */
    public Command setVelocityRPM(double velocityRPM) {
        return new InstantCommand(() -> {
            this.desiredOutput = velocityRPM;
            this.testMotor.setVelocity(velocityRPM);
            this.outputType = OutputType.kVelocity;
        });
    }

    /**
     * Set the position of the motor in rotations
     */
    public Command setPosition(double rotations) {
        return new InstantCommand(() -> {
            this.desiredOutput = rotations;
            this.testMotor.setPosition(rotations);
            this.outputType = OutputType.kPosition;
        });
    }

    @Override
    public void periodic() {
        this.logger.log("Desired Output", desiredOutput);
        this.logger.log("Output Type", outputType);
    }
}
