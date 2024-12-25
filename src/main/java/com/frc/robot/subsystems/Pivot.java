package com.frc.robot.subsystems;

import com.lib.team8592.MatchMode;
import com.lib.team8592.PIDProfile;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.*;

public class Pivot extends NewtonSubsystem {
    private DCMotor m_armGearbox = DCMotor.getNeoVortex(1);

    private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
            m_armGearbox, 
            200.0,
            SingleJointedArmSim.estimateMOI(0.5, 10), 
            Units.inchesToMeters(30), 
            Units.degreesToRadians(-75), 
            Units.degreesToRadians(255), 
            true, 
            0d
        );

    private Encoder m_encoder = new Encoder(1, 2);
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

    private final Mechanism2d m_mech2d = new Mechanism2d(1, 1);
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 0.5, 0.5);
    
    private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 0.5, -90));
    
    private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              0.5,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

    private PWMSparkFlex m_motor = new PWMSparkFlex(10);

    private PIDProfile ARM_GAINS = new PIDProfile().setP(50d);
    private PIDController m_controller = ARM_GAINS.toPIDController();

    protected Pivot(boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.m_encoder.setDistancePerPulse(2*Math.PI/4096);

        SmartDashboard.putData("Pivot", m_mech2d);
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_armSim.update(0.020);

        this.m_armTower.setColor(new Color8Bit(Color.kBlue));

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        m_encoderSim.setDistance(m_armSim.getAngleRads());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

        // Update the Mechanism Arm angle based on the simulated arm angle
        m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    }

    /** Run the control loop to reach and maintain the setpoint from the preferences. */
    public void reachSetpoint(double armSetpointDegrees) {
        var pidOutput =
            m_controller.calculate(
                m_encoder.getDistance(), Units.degreesToRadians(armSetpointDegrees));
        
        m_motor.setVoltage(pidOutput);
    }

    @Override
    public void onInit(MatchMode mode) {

    }

    @Override
    public void periodicLogs() {
        
    }

    @Override
    public void stop() {
        m_motor.set(0.0);
    }
}