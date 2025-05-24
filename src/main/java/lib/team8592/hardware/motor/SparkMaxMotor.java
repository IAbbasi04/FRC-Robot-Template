package lib.team8592.hardware.motor;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

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