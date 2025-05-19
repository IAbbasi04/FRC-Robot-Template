package org.team1731.lib.hardware.motor.spark;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMaxMotor extends SparkBaseMotor<SparkMax, SparkMaxConfig> {
    public SparkMaxMotor(int motorID) {
        this(motorID, false);
    }

    public SparkMaxMotor(int motorID, boolean inverted) {
        super(
            new SparkMax(motorID, MotorType.kBrushless), 
            new SparkMaxConfig(),
            inverted
        );
    }
}