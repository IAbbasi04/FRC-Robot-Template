package org.team8592.frc.robot.subsystems.superstructure.wrist;

import org.team8592.frc.robot.Constants;
import org.team8592.frc.robot.Robot;
import org.team8592.lib.PIDProfile;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOSim extends WristIO {
    private PIDProfile positionGains = new PIDProfile()
        .setP(1.0E-0)
        .setV(7.5E-3)
    ;
    
    private DCMotor wristMotor = DCMotor.getKrakenX60(1);

    private SingleJointedArmSim wristSim = new SingleJointedArmSim(
        wristMotor,
        Constants.WRIST.WRIST_GEAR_RATIO,
        0.005,
        0.05, // 5 cm
        // -Math.PI,
        // Math.PI,
        -Double.POSITIVE_INFINITY,
        Double.POSITIVE_INFINITY,
        false,
        0d
    );

    private double targetVoltage = 0d;

    private PIDController wristCtrl = positionGains.toPIDController();

    private ArmFeedforward wristFF = new ArmFeedforward(positionGains.kS, 0d, positionGains.kV);

    @Override
    public void setPercentOutput(double desiredPercent) {
        this.wristSim.setInput(12d * desiredPercent);
    }

    @Override
    public void setDegrees(double desiredDegrees) {
        this.targetDegrees = desiredDegrees;
        this.targetVoltage = wristCtrl.calculate(getDegrees(), getTargetDegrees()) + wristFF.calculate(targetDegrees, 0);
    }

    @Override
    public double getDegrees() {
        return Units.radiansToDegrees(wristSim.getAngleRads());
    }

    @Override
    public double getVoltage() {
        return getVelocityRPM() / 502.1;
    }

    @Override
    public double getTargetVoltage() {
        return this.targetVoltage;
    }

    @Override
    public double getVelocityRPM() {
        // return 0d;
        return wristSim.getVelocityRadPerSec() * 0.05 * 60d;
    }

    @Override
    public void updateInputs() {
        this.wristSim.update(Robot.CLOCK.dt());
    }
}