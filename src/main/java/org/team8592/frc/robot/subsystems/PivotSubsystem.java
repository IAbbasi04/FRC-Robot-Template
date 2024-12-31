package org.team8592.frc.robot.subsystems;

import org.team8592.frc.robot.Constants;
import org.team8592.frc.robot.Ports;
import org.team8592.lib.MatchMode;
import org.team8592.lib.PIDProfile;
import org.team8592.lib.hardware.motor.NewtonMotor;
import org.team8592.lib.hardware.motor.spark.SparkFlexMotor;

public class PivotSubsystem extends NewtonSubsystem {
    private NewtonMotor pivotMotor;

    private PIDProfile pivotGains = new PIDProfile()
        .setP(8E-4)
        .setV(1.6E-3)
        ;

    public enum PivotState {
        kManual(0d),
        kOverride(0d),
        kGridScore(10d),
        kGroundIntakeLower(0d),
        kGroundIntakeUpright(15d),
        kLowScore(20d),
        kHPIntake(25d),
        kHighScore(75d),
        kStow(90d),
        ;

        public final double degrees;
        private PivotState(double degrees) {
            this.degrees = degrees;
        }
    }

    private PivotState desiredPivotState = PivotState.kStow;

    private double desiredManualVelocityRPM = 0d;
    private double desiredOverrideDegrees = 0d;

    protected PivotSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.pivotMotor = new SparkFlexMotor(Ports.PIVOT_CAN_ID);
        this.pivotMotor.withGains(pivotGains);
    }

    public void setState(PivotState state) {
        this.desiredPivotState = state;
    }

    public void setVelocity(double desiredRPM) {
        if (!isEnabled()) return;
        
        this.desiredManualVelocityRPM = desiredRPM;
        this.desiredPivotState = PivotState.kManual;
    }

    public void setAngle(double desiredDegrees) {
        if (!isEnabled()) return;

        this.desiredOverrideDegrees = desiredDegrees;
        this.desiredPivotState = PivotState.kOverride;
    }

    private double fromMotorRotationsToPivotDegrees(double rot) {
        return (rot * 360d) / Constants.PIVOT.MOTOR_TO_PIVOT_GEAR_RATIO;
    }

    private double fromPivotDegreesToMotorRotations(double degrees) {
        return (degrees / 360d) * Constants.PIVOT.MOTOR_TO_PIVOT_GEAR_RATIO;
    }

    public double getMotorRotations() {
        return this.pivotMotor.getRotations();
    }

    public double getPivotDegrees() {
        return fromMotorRotationsToPivotDegrees(getMotorRotations());
    }

    @Override
    public void onInit(MatchMode mode) {}

    @Override
    public void periodicLogs() {
        this.logger.log("Desired Pivot State", this.desiredPivotState);
        this.logger.log("Current Pivot Degrees", getPivotDegrees());
        this.logger.log("Current Pivot Rotations", getMotorRotations());
        this.logger.log("Desired Pivot Degrees", this.desiredPivotState.degrees);
        this.logger.log("Desired Manual Velocity RPM", this.desiredManualVelocityRPM);
        this.logger.log("Desired Pivot Rotations", 
            fromPivotDegreesToMotorRotations(desiredPivotState.degrees)
        );
    }

    @Override
    public void periodicOutputs() {
        if (desiredPivotState == null || !isEnabled()) return; // Do nothing in either case

        switch(desiredPivotState) {
            case kManual:
                this.pivotMotor.setVelocity(this.desiredManualVelocityRPM);
                break;
            case kOverride:
                this.pivotMotor.setPositionSmartMotion(
                    fromPivotDegreesToMotorRotations(desiredOverrideDegrees)
                );
                break;
            default: // Preset positions
                this.pivotMotor.setPositionSmartMotion(
                    fromPivotDegreesToMotorRotations(desiredPivotState.degrees)
                );
                break;
        }
    }

    @Override
    public void stop() {
        this.setVelocity(0d);
    }
}