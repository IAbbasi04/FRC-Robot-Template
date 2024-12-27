package com.lib.team8592.hardware.motor;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SparkFlexMotorController extends SparkBaseMotorController<CANSparkFlex> {
    public SparkFlexMotorController(int motorID) {
        this(motorID, false);
    }

    public SparkFlexMotorController(int motorID, boolean reversed) {
        super(new CANSparkFlex(motorID, MotorType.kBrushless), reversed);
    }
}