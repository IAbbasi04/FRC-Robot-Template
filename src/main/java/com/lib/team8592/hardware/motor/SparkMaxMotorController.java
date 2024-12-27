package com.lib.team8592.hardware.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SparkMaxMotorController extends SparkBaseMotorController<CANSparkMax> {
    public SparkMaxMotorController(int motorID) {
        this(motorID, false);
    }

    public SparkMaxMotorController(int motorID, boolean reversed) {
        super(new CANSparkMax(motorID, MotorType.kBrushless), reversed);
    }
}