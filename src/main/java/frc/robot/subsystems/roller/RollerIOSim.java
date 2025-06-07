package frc.robot.subsystems.roller;

import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class RollerIOSim extends RollerIO {
    private DCMotorSim rollerSim;
    
    public RollerIOSim() {
        this.rollerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(1E-2, 0.02),
            DCMotor.getKrakenX60(1)
        );
    } 

    @Override
    public void setVelocity(double desiredRPM) {
        this.desiredRPM = desiredRPM;
        
        // Convert RPM to rad/s
        this.rollerSim.setAngularVelocity(desiredRPM * (2 * Math.PI / 60));
    }

    @Override
    public double getVelocityRPM() {
        return this.rollerSim.getAngularVelocityRPM();
    }

    @Override
    public void setPercentOutput(double desiredPercent) {
        this.rollerSim.setInput(desiredPercent * 12d);
    }

    @Override
    public double getOutputVoltage() {
        return this.rollerSim.getInputVoltage();
    }

    @Override
    public void updateInputs() {
        this.rollerSim.update(Robot.CLOCK.dt());
    }
}