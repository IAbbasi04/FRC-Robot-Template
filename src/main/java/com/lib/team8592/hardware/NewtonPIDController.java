package com.lib.team8592.hardware;

import java.util.ArrayList;

import com.lib.team8592.PIDProfile;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class NewtonPIDController implements Sendable {
    private ArrayList<PIDProfile> pidGains = new ArrayList<>();
    private ArrayList<ProfiledPIDController> pidCtrls = new ArrayList<>();

    private double setpoint = 0d;

    private static int instances;

    public NewtonPIDController(PIDProfile... pidGains) {
        int i = 0;
        for (PIDProfile gains : pidGains) {
            this.pidGains.add(i, gains);
            this.pidCtrls.add(i, gains.toProfiledPIDController());
            i++;
        }

        instances++;
        SendableRegistry.add(this, "NewtonPIDController", instances);
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double calculate(double measurement, double setpoint, double dt, int slot) {
        this.setSetpoint(setpoint);
        // SmartDashboard.putNumber("NNCCAAAKS;LAD;KLDL",
        //     pidGains.get(slot).feedForward.calculate(0, dt)
        // );

        // return pidCtrls.get(slot).calculate(measurement, setpoint) + 
        //     pidGains.get(slot).feedForward.calculate((setpoint-measurement) / dt, slot);

        // double p = pidGains.get(slot).kP * (setpoint - measurement);
        // double i = pidGains.get(slot).kI * (setpoint - measurement);
        return 0d;
    }

    public double calculate(double measurement, double setpoint, int slot) {
        return this.calculate(measurement, setpoint, 0.02, slot);
    }

    public double calculate(double measurement, double setpoint) {
        return this.calculate(measurement, setpoint, 0.02, 0);
    }

    public double calculate(double measurement, int slot) {
        return this.calculate(measurement, this.setpoint, 0.02, slot);
    }

    public double calculate(double measurement) {
        return this.calculate(measurement, this.setpoint, 0.02, 0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {}
}