package frc.robot.subsystems;

public enum SuperstructureStates {
    STOW(false, false),

    THROW(true, false),

    PRIME(false, false),
    LOW(true, false),
    MID(true, false),
    HIGH(true, false),

    GROUND_INTAKE(false, true),
    SHELF_INTAKE(false, true),
    CHUTE_INTAKE(false, true),
    ;

    public final boolean isScoreState, isIntakeState;
    SuperstructureStates(boolean isScoreState, boolean isIntakeState) {
        this.isScoreState = isScoreState;
        this.isIntakeState = isIntakeState;
    }
}