package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotSelector.RobotType;
import lib.team1731.MatchMode;

public class Suppliers {
    public static final BooleanSupplier IS_RED_ALLIANCE = () -> 
        DriverStation.getAlliance().isPresent() && 
        DriverStation.getAlliance().get() == Alliance.Red;

    public static final BooleanSupplier IS_SIMULATION = () -> Robot.isSimulation();

    public static final Supplier<MatchMode> CURRENT_MODE = () -> Robot.MODE;

    public static final Supplier<RobotType> CURRENT_ROBOT = () -> RobotSelector.getRobot();
}