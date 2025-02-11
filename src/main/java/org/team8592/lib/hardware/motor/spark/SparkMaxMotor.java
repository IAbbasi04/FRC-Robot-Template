package org.team8592.lib.hardware.motor.spark;

import org.team8592.lib.hardware.motor.MotorConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMaxMotor extends SparkBaseMotor<SparkMax, SparkMaxConfig> {
    public SparkMaxMotor(int motorID) {
        this(motorID, false);
    }

    public SparkMaxMotor(int motorID, boolean inverted) {
        super(new SparkMax(motorID, MotorType.kBrushless), inverted, new MotorConstants(
            5676d, 
            105d, 
            2.6,
            493.5
        ));
    }
}