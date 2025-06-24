package frc.robot.subsystems.superstructure;

public enum SuperState {
    HOME(),
    FLOOR(),
    L1(),
    L2(),
    L3(),
    L4(),
    ;

    public enum GamePiece {
        CORAL,
        ALGAE,
        CAGE;
    }

    public enum Action {
        STOW,
        INTAKE,
        SCORE,
        GO_TO
    }
}