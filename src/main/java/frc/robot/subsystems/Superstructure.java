package frc.robot.subsystems;

import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionConstants;
import lib.commands.WaitUntilCommand;

/**
 * Super class meant to represent the entire robot; Mainly used for commands used across subsystems
 */
public class Superstructure extends SubsystemBase {
    private SubsystemManager manager;
    public Superstructure(SubsystemManager manager) {
        this.manager = manager;
    }

    private static SuperstructureStates targetIntakeState = SuperstructureStates.GROUND_INTAKE;
    private static SuperstructureStates targetScoreState = SuperstructureStates.HIGH;
    private static SuperstructureStates appliedState = SuperstructureStates.STOW;

    public Command updateSwerveTelemetry() {
        return manager.vision.run(
            () -> {
                Optional<EstimatedRobotPose> estimatedRobotPose = manager.vision.getEstimatedRobotPose();
                if (estimatedRobotPose.isPresent()) {
                    Pose2d robotPose = estimatedRobotPose.get().estimatedPose.toPose2d();
                    double ambiguity = manager.vision.getPoseAmbiguity();

                    if(Math.abs(ambiguity) < VisionConstants.MAX_ACCEPTABLE_AMBIGUITY) {
                        if (DriverStation.isDisabled()){
                            manager.swerve.doOnce(manager.swerve.resetPose(robotPose));
                        } else {
                            manager.swerve.doOnce(manager.swerve.addVisionMeasurement(robotPose));
                        }
                    }
                }
            }
        )
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .onlyIf(() -> Robot.isReal())
        .ignoringDisable(true);
    }

    public Command setTargetState(SuperstructureStates desiredState) {
        return manager.run(() -> {
            if (desiredState.isIntakeState) {
                targetIntakeState = desiredState;
            } else if (desiredState.isScoreState) {
                targetScoreState = desiredState;
            } else {
                appliedState = desiredState;
            }
        });
    }

    public Command applyTargetIntakeState() {
        return this.applyState(targetIntakeState);
    }

    public Command applyTargetScoreState() {
        return this.applyState(targetScoreState);
    }

    public Command applyState(SuperstructureStates desiredState) {
        return new InstantCommand(() -> appliedState = desiredState).andThen(applyState());
    }

    public Command applyState() {
        return new DeferredCommand(() -> {
            switch(appliedState) {
                case STOW:
                    return manager.wrist.setWristDegrees(0.0).alongWith(
                        manager.elevator.setHeightInches(0.0),
                        new WaitUntilCommand(manager.pivot.setRotations(0.0), () -> manager.elevator.getHeightInches() < 1.0)
                    );
                case THROW:
                    return manager.wrist.setWristDegrees(120.0).alongWith(
                        new WaitUntilCommand(scorePiece(), () -> manager.wrist.getDegrees() > 45.0)
                    );
                case PRIME:
                    return manager.pivot.setRotations(20.0).alongWith(
                        manager.elevator.setHeightInches(0.0),
                        manager.wrist.setWristDegrees(0.0)
                    );
                case LOW:
                    return manager.wrist.setWristDegrees(45.0).alongWith(
                        manager.elevator.setHeightInches(0.0),
                        new WaitUntilCommand(manager.pivot.setRotations(0.0), () -> manager.elevator.getHeightInches() < 1.0)
                    );
                case MID:
                    return manager.pivot.setRotations(20.0)
                        .alongWith(
                            new WaitUntilCommand(
                                manager.elevator.setHeightInches(6.0).alongWith(
                                    manager.wrist.setWristDegrees(135.0)
                                ), 
                                () -> manager.pivot.getRotations() > 18.0
                            )
                        );
                case HIGH:
                    return manager.pivot.setRotations(20.0).alongWith(
                        new WaitUntilCommand(
                            manager.elevator.setHeightInches(12.0).alongWith(
                                manager.wrist.setWristDegrees(135.0)
                            ), 
                            () -> manager.pivot.getRotations() > 18.0
                        )
                    );
                case GROUND_INTAKE:
                    return manager.wrist.setWristDegrees(120.0).alongWith(
                        manager.elevator.setHeightInches(0.0),
                        new WaitUntilCommand(manager.pivot.setRotations(0.0), () -> manager.elevator.getHeightInches() < 1.0)
                    );
                case SHELF_INTAKE:
                    return manager.pivot.setRotations(20.0).alongWith(
                        new WaitUntilCommand(
                            manager.elevator.setHeightInches(8.0).alongWith(
                                manager.wrist.setWristDegrees(120.0)
                            ), 
                            () -> manager.pivot.getRotations() > 18.0
                        )
                    );
                case CHUTE_INTAKE:
                    return manager.pivot.setRotations(20.0).alongWith(
                        manager.elevator.setHeightInches(0.0),
                        manager.wrist.setWristDegrees(0.0)
                    );
                default:
                    return Commands.none();
            }
        }, Set.of(manager.wrist, manager.elevator, manager.pivot));
    }

    public Command intakePiece() {
        return manager.rollers.setRollerVelocity(5000.0);
    }

    public Command scorePiece() {
        return manager.rollers.setRollerVelocity(-3000.0);
    }
}