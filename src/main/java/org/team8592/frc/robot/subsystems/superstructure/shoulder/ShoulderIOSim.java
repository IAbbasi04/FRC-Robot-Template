package org.team8592.frc.robot.subsystems.superstructure.shoulder;

import org.team8592.frc.robot.Robot;
import org.team8592.lib.PIDProfile;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShoulderIOSim extends ShoulderIO {
    private SingleJointedArmSim shoulderSim;

    private Mechanism2d shoulder;
    private MechanismRoot2d root;
    private MechanismLigament2d base;
    private MechanismLigament2d arm;

    private ProfiledPIDController positionCtrl = new PIDProfile()
        .setP(3.0)
        .setMaxVelocity(360d)
        .setMaxAcceleration(720d)
        .toProfiledPIDController()
        ;

    public ShoulderIOSim() {
        this.shoulderSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), 
            180d, 
            (1d/12d * Units.lbsToKilograms(10) * Units.inchesToMeters(100)),
            Units.inchesToMeters(20), 
            Units.degreesToRadians(-20), 
            Units.degreesToRadians(225), 
            false, 
            0
        );

        this.shoulder = new Mechanism2d(1d, 1d);
        this.root = this.shoulder.getRoot("Shoulder Root", 0.1, 0.1);
        this.base = this.root.append(new MechanismLigament2d("Base", 0.5, 90));
        this.arm = this.base.append(new MechanismLigament2d("Arm", Units.inchesToMeters(20), -135));

        SmartDashboard.putData("Shoulder Sim", this.shoulder);
    }

    @Override
    public void setDegrees(double degrees) {
        double voltage = this.positionCtrl.calculate(getDegrees(), degrees);
        this.shoulderSim.setInput(voltage);
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
        this.shoulderSim.update(Robot.CLOCK.dt());
        this.arm.setAngle(Units.radiansToDegrees(this.shoulderSim.getAngleRads()) - 135d);
    }
}