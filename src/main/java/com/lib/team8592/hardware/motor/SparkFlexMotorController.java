package com.lib.team8592.hardware.motor;

import com.lib.team8592.PIDProfile;
import com.lib.team8592.Utils;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

public class SparkFlexMotorController extends SparkBaseMotorController<CANSparkFlex> {
    public SparkFlexMotorController(int motorID) {
        this(motorID, false);
    }

    public SparkFlexMotorController(int motorID, boolean reversed) {
        super(new CANSparkFlex(motorID), reversed);
        this.motor = new CANSparkFlex(motorID, MotorType.kBrushless);
        this.motorCtrl = motor.getPIDController();
        this.encoder = motor.getEncoder();

        this.motor.setInverted(reversed);
    }
}