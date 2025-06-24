package frc.robot.subsystems.superstructure;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.superstructure.Superstructure.*;

public class SequenceBuilder {
    private SubsystemManager manager;

    public SequenceBuilder(SubsystemManager manager) {
        this.manager = manager;
    }

    public Command buildSequence(Height height, Piece piece) {
        return Commands.none();
    }

    private Command goToPosition(double elev, double pivot, double wrist) {
        return new DeferredCommand(
            () -> {
                double curElev = 0d;
                double curPivot = 0d;
                double curWrist = 0d;

                double targetElev = elev;
                double targetPivot = pivot;
                double targetWrist = wrist;
            
                double elevRaisePivotThreshold = 0;
                if (curPivot < elevRaisePivotThreshold) { // if pivot under certain threshold do not raise elevator
                    targetElev = Math.min(targetElev, elevRaisePivotThreshold);
                }

                
                double pivotLowerElevThreshold = 0;
                if (curElev > pivotLowerElevThreshold) { // if elevator extended beyond certain threshold do not lower pivot below threshold
                    targetPivot = Math.max(targetPivot, pivotLowerElevThreshold);
                }

                double pivotElevLowerWristThreshold = 0;
                if (curWrist > pivotElevLowerWristThreshold) { // if wrist extended beyond certain threshold do not lower pivot below threshold or elev under threshold depending on what is extended (or both?)
                    targetPivot = Math.max(targetPivot, pivotElevLowerWristThreshold);
                    targetElev = Math.max(targetElev, pivotElevLowerWristThreshold);
                }

                double pivotRaiseWristThreshold = 0;
                double elevRaiseWristThreshold = 0;

                if (curPivot < pivotRaiseWristThreshold || curElev < elevRaiseWristThreshold) { // If pivot or elevator under certain threshold do not move wrist
                    targetWrist = curWrist;
                }

                // Move all subsystems simultaneously to the target positions
                return Commands.none();
            }, 
            Set.of() // Make sure all subsystems are added as requirements
        );
    }
}