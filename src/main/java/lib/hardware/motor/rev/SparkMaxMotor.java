package lib.hardware.motor.rev;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import lib.hardware.motor.PortConfig;

/**
 * Wrapper class for motors that use the Spark Max motor controller
 */
public class SparkMaxMotor extends SparkBaseMotor<SparkMax, SparkMaxConfig> {
    public SparkMaxMotor(PortConfig config) {
        super(
            config.kBus,
            new SparkMax(config.kPort, MotorType.kBrushless), 
            new SparkMaxConfig(),
            config.kInverted
        );
    }
}