package frc.robot.helpers.motor.talonfx;

import frc.robot.helpers.motor.MotorConstants;

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

        super.positionOutput.withEnableFOC(true);
        super.velocityOutput.withEnableFOC(true);
        super.percentOutput.withEnableFOC(true);
        super.motionMagicOutput.withEnableFOC(true);
    }
}