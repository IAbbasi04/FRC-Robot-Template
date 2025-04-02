package org.team8592.lib.hardware.motor.spark;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.team8592.lib.hardware.motor.MotorConstants;

public class NeoMotor extends SparkBaseMotor<SparkMax, SparkMaxConfig> {
    public NeoMotor(int motorID) {
        this(motorID, false);
    }

    public NeoMotor(int motorID, boolean inverted) {
        super(
            new SparkMax(motorID, MotorType.kBrushless), 
            new SparkMaxConfig(),
            inverted, 
            new MotorConstants(
                5676d, 
                105d, 
                2.6,
                493.5
            )
        );
    }
}