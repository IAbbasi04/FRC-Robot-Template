package org.team8592.lib.hardware.motor.spark;

import org.team8592.lib.hardware.motor.MotorConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SparkMaxMotor extends SparkBaseMotor<CANSparkMax> {
    public SparkMaxMotor(int motorID) {
        this(motorID, false);
    }

    public SparkMaxMotor(int motorID, boolean inverted) {
        super(new CANSparkMax(motorID, MotorType.kBrushless), inverted, new MotorConstants(
            5676d, 
            105d, 
            2.6,
            493.5
        ));
    }
}