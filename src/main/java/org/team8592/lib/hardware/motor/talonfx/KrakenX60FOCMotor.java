package org.team8592.lib.hardware.motor.talonfx;

import org.team8592.lib.hardware.motor.MotorConstants;

public class KrakenX60FOCMotor extends TalonFXMotor {
    public KrakenX60FOCMotor(int motorID) {
        this(motorID, false);
    }

    public KrakenX60FOCMotor(int motorID, boolean inverted) {
        super(motorID, inverted, new MotorConstants(
            5800d, 
            483d, 
            9.37,
            484.8
        ));
    }
}