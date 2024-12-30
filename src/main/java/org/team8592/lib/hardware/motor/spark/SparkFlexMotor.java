package com.lib.team8592.hardware.motor.spark;

import com.lib.team8592.hardware.motor.MotorConstants;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SparkFlexMotor extends SparkBaseMotor<CANSparkFlex> {
    public SparkFlexMotor(int motorID) {
        this(motorID, false);
    }

    public SparkFlexMotor(int motorID, boolean inverted) {
        super(new CANSparkFlex(motorID, MotorType.kBrushless), inverted, new MotorConstants(
            6784d, 
            211d, 
            3.6, 
            575.1
        ));
    }
}