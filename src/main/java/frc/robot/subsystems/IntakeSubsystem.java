package frc.robot.subsystems;

import frc.robot.config.Ports;
import lib.MatchMode;
import lib.hardware.motor.ctre.TalonFXMotor;
import lib.subsystem.BaseSubsystem;

public class IntakeSubsystem extends BaseSubsystem {
    private TalonFXMotor coralMotor, algaeMotor;

    public IntakeSubsystem() {
        this.coralMotor = new TalonFXMotor(Ports.CAN.INTAKE_CORAL);
        this.algaeMotor = new TalonFXMotor(Ports.CAN.INTAKE_ALGAE);
    }

    @Override
    public void onModeInit(MatchMode mode) {
    }

    @Override
    public void periodicTelemetry() {
    }

    @Override
    public void stop() {
        
    }
}