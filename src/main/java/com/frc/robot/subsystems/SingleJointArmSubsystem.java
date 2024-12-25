package com.frc.robot.subsystems;

import com.lib.team8592.MatchMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public abstract class SingleJointArmSubsystem extends NewtonSubsystem {
    protected DCMotor motorGearbox;
    protected ArmConstants constants;
    protected SingleJointedArmSim armSim;
    protected Encoder encoder;
    protected EncoderSim encoderSim;

    protected SingleJointArmSubsystem(DCMotor motorGearbox, ArmConstants constants, Encoder encoder, boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.motorGearbox = motorGearbox;
        this.armSim = new SingleJointedArmSim( 
            this.motorGearbox,
            constants.gearboxRatio,
            SingleJointedArmSim.estimateMOI(
                constants.armLengthMeters, 
                constants.armMassKg
            ),
            constants.armLengthMeters,
            Units.degreesToRadians(constants.minAngleDegrees),
            Units.degreesToRadians(constants.maxAngleDegrees),
            true,
            Units.degreesToRadians(constants.startAngleDegrees)
        );

        this.encoder = encoder;
        this.encoderSim = new EncoderSim(encoder);
    }

    @Override
    public void onInit(MatchMode mode) {}

    @Override
    public void periodicLogs() {}

    @Override
    public void stop() {}

    protected class ArmConstants {
        public double gearboxRatio = 0d;
        public double armLengthMeters = 0d;
        public double armMassKg = 0d;
        public double minAngleDegrees = 0d;
        public double maxAngleDegrees = 0d;
        public double startAngleDegrees = 0d;

        public ArmConstants(double gear, double length, double mass, double minAngle, double maxAngle, double startAngle) {
            this.gearboxRatio = gear;
            this.armLengthMeters = length;
            this.armMassKg = mass;
            this.minAngleDegrees = minAngle;
            this.maxAngleDegrees = maxAngle;
            this.startAngleDegrees = startAngle;
        }
    }
}