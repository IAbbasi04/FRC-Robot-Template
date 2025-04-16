package org.team8592.lib.hardware.motor;

public class MotorConstants {
    public final double MAX_VELOCITY_RPM;
    public final double STALL_CURRENT_AMPS;
    public final double STALL_TORQUE_NM;
    public final double MOTOR_KV;

    private MotorConstants(double maxRPM, double stallAmp, double stallTorque, double motorKV) {
        this.MAX_VELOCITY_RPM = maxRPM;
        this.STALL_CURRENT_AMPS = stallAmp;
        this.STALL_TORQUE_NM = stallTorque;
        this.MOTOR_KV = motorKV;
    }

    public class VortexConstants extends MotorConstants {
        public VortexConstants() {
            super(6784d, 
            211d, 
            3.6, 
            575.1);
        }
    }

    public class NeoConstants extends MotorConstants {
        public NeoConstants() {
            super(5676d, 
            105d, 
            2.6,
            493.5);
        }
    }

    public class KrakenX60Constants extends MotorConstants {
        public KrakenX60Constants() {
            // TODO - FIX THESE
            super(
                0.0, 
                0.0, 
                0.0,
                0.0
            );
        }
    }

    public class KrakenX44Constants extends MotorConstants {
        public KrakenX44Constants() {
            // TODO - FIX THESE
            super(
                0.0, 
                0.0, 
                0.0,
                0.0
            );
        }
    }
}