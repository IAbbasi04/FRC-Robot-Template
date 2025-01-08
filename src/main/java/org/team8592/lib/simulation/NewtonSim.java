package org.team8592.lib.simulation;

import org.team8592.lib.logging.SmartLogger;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public abstract class NewtonSim<S extends LinearSystemSim<N1, N1, N1>> {
    protected S simulation;
    protected DCMotorSim simMotor;
    protected EncoderSim simEncoder;
    protected DCMotorSim gearbox;
    protected SmartLogger logger;

    public NewtonSim(S simulation, DCMotorSim gearbox) {
        this.simulation = simulation;
        this.gearbox = gearbox;
        this.logger = new SmartLogger(this.getClass().getSimpleName());
    }

    public void update(double dt) {
        simulation.update(dt);
        updateSimulation(dt);
    }

    public abstract void updateSimulation(double dt);

    public double getCurrentDraw() {
        return simulation.getCurrentDrawAmps();
    }
}