package org.team1731.frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.team1731.frc.robot.RobotSelector.RobotType;
import org.team1731.lib.MatchMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Suppliers {
    public static final BooleanSupplier IS_RED_ALLIANCE = () -> 
        DriverStation.getAlliance().isPresent() && 
        DriverStation.getAlliance().get() == Alliance.Red;

    public static final BooleanSupplier IS_SIMULATION = () -> Robot.isSimulation();

    public static final Supplier<MatchMode> CURRENT_MODE = () -> Robot.MODE;

    public static final Supplier<RobotType> CURRENT_ROBOT = () -> RobotSelector.getRobot();
}