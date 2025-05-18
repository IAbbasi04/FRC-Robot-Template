package org.team8592.frc.robot.subsystems.superstructure.wrist;

import org.team8592.frc.robot.Constants;
import org.team8592.frc.robot.Constants.SHOULDER;
import org.team8592.frc.robot.Constants.SUPERSTRUCTURE;
import org.team8592.frc.robot.Constants.WRIST;
import org.team8592.frc.robot.Robot;
import org.team8592.frc.robot.subsystems.superstructure.Superstructure;
import org.team8592.lib.PIDProfile;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristIOSim extends WristIO {    
    private SingleJointedArmSim wristSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1),
        1d / WRIST.WRIST_GEAR_RATIO,
        WRIST.WRIST_MOMENT_OF_INERTIA,
        SUPERSTRUCTURE.WRIST_LENGTH_METERS, // 5 cm
        WRIST.WRIST_ANGLE_DEGREES_MIN,
        WRIST.WRIST_ANGLE_DEGREES_MAX,
        false,
        0d
    );

    private ProfiledPIDController positionCtrl = new PIDProfile()
        .setP(1.0)
        .setMaxVelocity(360d)
        .setMaxAcceleration(720d)
        .setTolerance(WRIST.WRIST_POSITION_TOLERANCE_DEGREES)
        .toProfiledPIDController()
    ;

    @Override
    public void setPercentOutput(double desiredPercent) {
        this.wristSim.setInput(12d * desiredPercent);
    }

    @Override
    public void setDegrees(double degrees) {
        double voltage = this.positionCtrl.calculate(getDegrees(), degrees);
        SmartDashboard.putNumber("ZZAJKLSDJKLADJK", voltage);
        this.wristSim.setInput(voltage);
    }

    @Override
    public double getDegrees() {
        return Units.radiansToDegrees(wristSim.getAngleRads());
    }

    @Override
    public void updateInputs() {
        this.wristSim.update(Robot.CLOCK.dt());
        Superstructure.wristLigament.setAngle(Units.radiansToDegrees(this.wristSim.getAngleRads()) + SUPERSTRUCTURE.WRIST_OFFSET_DEGREES);
    }
}