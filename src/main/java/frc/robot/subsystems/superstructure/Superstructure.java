package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SubsystemManager;

public class Superstructure {
    private SubsystemManager manager;
    private SequenceBuilder builder;

    protected Height targetHeight = Height.START;
    protected Piece targetPiece = Piece.CORAL;

    public enum Height {
        START,
        HOME,
        FLOOR,
        L1,
        L2,
        L3,
        L4
    }

    public enum Piece {
        CORAL,
        ALGAE,
        CAGE;
    }

    public Superstructure(SubsystemManager manager) {
        this.manager = manager;
        this.builder = new SequenceBuilder(manager);
    }

    public Command buildSequence(Height height, Piece piece) {
        return builder.buildSequence(height, piece);
    }

    public Command setTargetHeight(Height height) {
        return new InstantCommand(() -> this.targetHeight = height);
    }

    public Command setTargetPiece(Piece piece) {
        return new InstantCommand(() -> this.targetPiece = piece);
    }

    public Command applyTargetPosition() {
        return Commands.run(() -> {
            
        }, new Subsystem[0]);
    }
}