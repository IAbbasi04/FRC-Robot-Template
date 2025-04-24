package org.team8592.frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.team8592.frc.robot.RobotSelector.RobotType;
import org.team8592.frc.robot.subsystems.superstructure.Superstructure;
import org.team8592.frc.robot.subsystems.superstructure.Superstructure.GamePiece;
import org.team8592.lib.MatchMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Suppliers {
    public static final BooleanSupplier IS_RED_ALLIANCE = () -> 
        DriverStation.getAlliance().isPresent() && 
        DriverStation.getAlliance().get() == Alliance.Red;

    public static final BooleanSupplier IS_SIMULATION = () -> Robot.isSimulation();

    public static final Supplier<MatchMode> CURRENT_MODE = () -> Robot.MODE;

    public static final Supplier<RobotType> CURRENT_ROBOT = () -> RobotSelector.getRobot();

    public static final Supplier<GamePiece> TARGET_PIECE = () -> Superstructure.getTargetPiece();
    public static final BooleanSupplier IS_CORAL_MODE = () -> TARGET_PIECE.get().equals(GamePiece.CORAL);
    public static final BooleanSupplier IS_ALGAE_MODE = () -> TARGET_PIECE.get().equals(GamePiece.ALGAE);
    public static final BooleanSupplier IS_CLIMB_MODE = () -> TARGET_PIECE.get().equals(GamePiece.CAGE);
}