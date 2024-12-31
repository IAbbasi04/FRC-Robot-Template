package org.team8592.frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.team8592.frc.robot.Suppliers;
import org.team8592.frc.robot.subsystems.SubsystemManager;
import org.team8592.frc.robot.subsystems.PivotSubsystem.PivotState;
import org.team8592.frc.robot.subsystems.SwerveSubsystem.DriveModes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public final class NewtonCommands {
    private static SubsystemManager manager;

    public static void initialize(SubsystemManager manager) {
        NewtonCommands.manager = manager;
    }

    /**
     * Command to drive the swerve with translation processed for human input and
     * rotation controlled by the snap-to PID controller (snapping to the passed-in)
     * angle
     *
     * @param angle the angle to snap to
     * @param driveX a lambda returning the driver's X input
     * @param driveY a lambda returning the driver's Y input
     *
     * @return the command
     */
    public static Command swerveSnapToCommand(Rotation2d angle, DoubleSupplier driveX, DoubleSupplier driveY){
        return manager.getSwerve().run(() -> {
            ChassisSpeeds processed = manager.getSwerve().processJoystickInputs(
                driveX.getAsDouble(),
                driveY.getAsDouble(),
                0
            );
            processed.omegaRadiansPerSecond = 
                manager.getSwerve().snapToAngle(
                    Suppliers.currentGyroscopeRotationOffset.get()
                    .plus(angle)
                );
            manager.getSwerve().drive(processed, DriveModes.AUTOMATIC);
        });
    }

    /**
     * Command to drive the swerve with translation processed for human input and
     * rotation controlled by the snap-to PID controller (snapping to the passed-in)
     * angle
     *
     * @param angle the angle to snap to
     * @param driveX a lambda returning the driver's X input
     * @param driveY a lambda returning the driver's Y input
     *
     * @return the command
     */
    public static Command swerveSnapToCommand(Rotation2d angle, DoubleSupplier driveX, DoubleSupplier driveY, boolean isRunningOnRed){
        Rotation2d allianceAngle = Suppliers.currentGyroscopeRotationOffset.get().plus(angle);
        return swerveSnapToCommand(allianceAngle, driveX, driveY);
    }

    public static Command setIntakeVelocityCommand(DoubleSupplier topRollerRPM, DoubleSupplier bottomRollerRPM) {
        return manager.getIntake().run(() -> {
            manager.getIntake().setRollerVelocity(topRollerRPM.getAsDouble(), bottomRollerRPM.getAsDouble());
        });
    }

    public static Command setPivotStateCommand(PivotState pivotState) {
        return manager.getPivot().run(() -> {
            manager.getPivot().setState(pivotState);
        });
    }

    public static Command setPivotVelocityCommand(DoubleSupplier pivotVelocityRPM) {
        return manager.getPivot().run(() -> {
            manager.getPivot().setVelocity(pivotVelocityRPM.getAsDouble());
        });
    }

    public static Command setRollersScoringCommand() {
        return setIntakeVelocityCommand(() -> -3000d, () -> -3000d);
    }

    public static Command setRollersSpittingCommand() {
        return setIntakeVelocityCommand(() -> -2000d, () -> -2000d);
    }

    public static Command setGroundIntakingCommand() {
        return setPivotStateCommand(PivotState.kGroundIntakeLower).alongWith(
            setIntakeVelocityCommand(() -> 3000d, () -> 3000d)
        );
    }

    public static Command setUprightIntakingCommand() {
        return setPivotStateCommand(PivotState.kGroundIntakeUpright).alongWith(
            setIntakeVelocityCommand(() -> 3000d, () -> 3000d)
        );
    }

    public static Command setHPIntakingCommand() {
        return setPivotStateCommand(PivotState.kHPIntake).alongWith(
            setIntakeVelocityCommand(() -> 3000d, () -> 3000d)
        );
    }

    public static Command primeGridScoreCommand() {
        return setPivotStateCommand(PivotState.kGridScore);
    }

    public static Command primeLowScoreCommand() {
        return setPivotStateCommand(PivotState.kLowScore);
    }

    public static Command primeHighScoreCommand() {
        return setPivotStateCommand(PivotState.kHighScore);
    }

    public static Command setStowStateCommand() {
        return setPivotStateCommand(PivotState.kStow).alongWith(
            manager.getIntake().getStopCommand()
        );
    }
}