package org.team8592.frc.robot.subsystems.superstructure.shoulder;

import org.team8592.frc.robot.Robot;
import org.team8592.frc.robot.Constants.SHOULDER;
import org.team8592.frc.robot.Constants.SUPERSTRUCTURE;
import org.team8592.frc.robot.subsystems.superstructure.Superstructure;
import org.team8592.lib.PIDProfile;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShoulderIOSim extends ShoulderIO {
    private SingleJointedArmSim shoulderSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1), 
        1d / SHOULDER.SHOULDER_GEAR_RATIO, 
        SHOULDER.SHOULDER_MOMENT_OF_INERTIA,
        SUPERSTRUCTURE.SHOULDER_LENGTH_METERS,
        SHOULDER.SHOULDER_MIN_DEGREES, 
        SHOULDER.SHOULDER_MAX_DEGREES,
        false,
        0d
    );

    private ProfiledPIDController positionCtrl = new PIDProfile()
        .setP(1.0)
        .setMaxVelocity(360d)
        .setMaxAcceleration(720d)
        .setTolerance(SHOULDER.SHOULDER_POSITION_TOLERANCE_DEGREES)
        .toProfiledPIDController()
        ;

    @Override
    public void setDegrees(double degrees) {
        // double voltage = this.positionCtrl.calculate(getDegrees(), degrees);
        // this.shoulderSim.setInput(voltage);
    }

    @Override
    public double getDegrees() {
        return Units.radiansToDegrees(this.shoulderSim.getAngleRads());
    }

    @Override
    public void setPercentOutput(double percent) {
        this.shoulderSim.setInput(12d * percent);
    }
    
    @Override
    public void updateInputs() {
        // this.shoulderSim.update(Robot.CLOCK.dt());
        // Superstructure.shoulderLigament.setAngle(Units.radiansToDegrees(this.shoulderSim.getAngleRads()) + SUPERSTRUCTURE.SHOULDER_OFFSET_DEGREES);
    }
}