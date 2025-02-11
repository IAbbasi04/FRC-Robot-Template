package org.team8592.lib.hardware.motor.spark;

import org.team8592.lib.hardware.motor.MotorConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

public class SparkFlexMotor extends SparkBaseMotor<SparkFlex, SparkFlexConfig> {
    public SparkFlexMotor(int motorID) {
        this(motorID, false);
    }

    public SparkFlexMotor(int motorID, boolean inverted) {
        super(
            new SparkFlex(motorID, MotorType.kBrushless), 
            new SparkFlexConfig(),
            inverted,
            new MotorConstants(
                6784d, 
                211d, 
                3.6, 
                575.1
            )
        );
    }
}