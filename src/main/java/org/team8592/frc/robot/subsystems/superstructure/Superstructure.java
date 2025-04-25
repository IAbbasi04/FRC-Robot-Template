package org.team8592.frc.robot.subsystems.superstructure;

import org.team8592.frc.robot.*;
import org.team8592.frc.robot.Constants.SHOULDER;
import org.team8592.frc.robot.RobotSelector.RobotType;
import org.team8592.frc.robot.subsystems.SubsystemManager;
import org.team8592.frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import org.team8592.frc.robot.subsystems.superstructure.shoulder.ShoulderSubsystem;
import org.team8592.frc.robot.subsystems.superstructure.wrist.WristSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.*;

public class Superstructure {
    // TODO - Put mech2d here
    public static Mechanism2d superstructure = new Mechanism2d(1d, 1d);
    public static MechanismRoot2d root = superstructure.getRoot("Superstructure", 0, 0);
    // public static MechanismLigament2d elevatorLigament = root.append(new MechanismLigament2d("Elevator", 0.5, 0, 0));
    
    public static GamePiece targetPiece = GamePiece.CORAL; // Which game piece the robot is targetting

    public static States targetState = States.START; // The user selected state to go to once the button is pressed
    public static States appliedState = States.START; // The position the robot is actively moving towards

    private static ElevatorSubsystem elevator;
    private static ShoulderSubsystem shoulder;
    private static WristSubsystem wrist;

    public static void addSubsystems(SubsystemManager manager) {
        Superstructure.elevator = manager.elevator;
        Superstructure.shoulder = manager.shoulder;
        Superstructure.wrist = manager.wrist;
    }

    public enum States {
        START(0, 0, 0, 0, 0, 0),
        STOW(0, 0, 0, 0, 0, 0),
        L1(0, 0, 0, 0, 0, 0),
        L2(0, 0, 0, 0, 0, 0),
        L3(0, 0, 0, 0, 0, 0),
        L4(0, 0, 0, 0, 0, 0),

        GROUND_ALGAE(0, 0, 0, 0, 0, 0),
        PROCESSOR(0, 0, 0, 0, 0, 0),
        L2_ALGAE(0, 0, 0, 0, 0, 0),
        L3_ALGAE(0, 0, 0, 0, 0, 0),
        NET(0, 0, 0, 0, 0, 0),

        DEEP_CLIMB_CLEARANCE(0, 0, 0, 0, 0, 0),
        ;

        public double wristDegrees, shoulderDegrees, elevatorInches;
        private States(double compWrist, double compShoulder, double compElevator, double pracWrist, double pracShoulder, double pracElevator){
            if(Suppliers.CURRENT_ROBOT.get().isCompBot() || Suppliers.CURRENT_ROBOT.get().equals(RobotType.SIM_BOT)) {
                this.wristDegrees = compWrist;
                this.shoulderDegrees = compShoulder;
                this.elevatorInches = compElevator;
            } else {
                this.wristDegrees = pracWrist;
                this.shoulderDegrees = pracShoulder;
                this.elevatorInches = pracElevator;
            }
        }
    }

    public enum GamePiece {
        CORAL,
        ALGAE,
        CAGE;
    }

    public static GamePiece getTargetPiece() {
        return targetPiece;
    }

    public static Command setTargetGamePiece(GamePiece targetPiece) {
        return Commands.runOnce(() -> {
            Superstructure.targetPiece = targetPiece;
        });
    }

    public static Command setTargetPosition(States targetState) {
        return Commands.runOnce(() -> Superstructure.targetState = targetState);
    }

    public static Command goToTargetPosition() {
        return goToPosition(targetState);
    }

    public static Command goToPosition(States state) {
        return Commands.runOnce(() -> appliedState = state).andThen(new ParallelCommandGroup(
            wrist
                .setDegrees(() -> wrist.getDegrees()) 
                .onlyWhile(
                    () -> (shoulder.getDegrees() < (SHOULDER.SAFE_SHOULDER_TO_ROTATE_WRIST_DEGREES - 10))
                ).andThen(
                    // If moving to net position 
                    // set wrist to -20 until elevator is at position
                    // then move wrist to target position
                    new ConditionalCommand(
                        wrist.setDegrees(() -> -20)
                            .until(() -> elevator.atPosition())
                            .andThen(wrist.setDegrees(() -> state.wristDegrees)),
                        // If moving to algae L3 position 
                        // halt wrist until shoulder is at position
                        // then move wrist to target position
                        new ConditionalCommand(
                            wrist.setDegrees(() -> wrist.getDegrees())
                                .until(() -> shoulder.atPosition())
                                .andThen(wrist.setDegrees(() -> state.wristDegrees)),
                            wrist.setDegrees(() -> state.wristDegrees), // Move the wrist to target position if none of the above conditions apply
                            () -> appliedState.equals(States.L3_ALGAE)
                        ),
                        () -> appliedState.equals(States.NET)
                    )
                ),
            
            // Halt elevator until shoulder is safely extended
            // then move elevator to target position
            elevator
                .setInches(() -> elevator.getInches()) 
                .onlyWhile(
                    () -> (shoulder.getDegrees() < (SHOULDER.SAFE_SHOULDER_TO_ROTATE_WRIST_DEGREES - 10))
                ).andThen( 
                    elevator.setInches(() -> state.elevatorInches)
                ),

            // Keep shoulder extended past safe extension point 
            // until wrist is at target position
            // then move shoulder to target position
            shoulder
                .setDegrees(() -> Math.max(SHOULDER.SAFE_SHOULDER_TO_ROTATE_WRIST_DEGREES, appliedState.shoulderDegrees)) 
                .onlyWhile(
                    () -> (!wrist.atPosition(appliedState.wristDegrees))
                ).andThen( // Move shoulder to target position
                    shoulder.setDegrees(() -> state.shoulderDegrees)
                )
        ));
    }
}