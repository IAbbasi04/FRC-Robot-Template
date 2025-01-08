package org.team8592.lib.simulation;

import org.team8592.lib.PIDProfile;
import org.team8592.lib.Utils;
import org.team8592.lib.hardware.NewtonFeedForward;
import org.team8592.lib.logging.LogUtils;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;

public class NewtonRollerSim extends NewtonSim<FlywheelSim> {
    private double desiredVelocityRPM = 0d;
    private double gearRatio = 1d;
    private double wheelRadius = 0d;
    private NewtonFeedForward feedForward = new NewtonFeedForward();
    private PIDProfile pidGains = new PIDProfile();

    public NewtonRollerSim(DCMotor gearbox, NewtonFeedForward feedForward, PIDProfile pidProfile, double gearing, double massKg, double radiusMeters) {
        super(
            new FlywheelSim(gearbox, gearing, Utils.getMOIForRoller(massKg, radiusMeters)), 
            new DCMotorSim(gearbox, gearing, Utils.getMOIForRoller(massKg, radiusMeters))
        );

        this.feedForward = feedForward;
        this.pidGains = pidProfile;

        this.gearRatio = gearing;
        this.wheelRadius = radiusMeters;

        RoboRioSim sim = RoboRioSim.get 

        // MechanismObject2d flywheel = MechanismObject2d
    }

    public void setVelocity(double desiredRPM) {
        this.desiredVelocityRPM = desiredRPM;
    }

    public double getDesiredVelocityRPM() {
        return this.desiredVelocityRPM;
    }

    public double getCurrentVelocityRPM() {
        return this.gearbox.getAngularVelocityRPM();
    }

    @Override
    public void updateSimulation(double dt) {

    }
}