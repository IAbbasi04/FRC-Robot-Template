package lib.team1731.sysid;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Deprecated (forRemoval = false)
public abstract class SysID {
    protected SysIdRoutine routine;
    public SysID(SysIdRoutine routine) {
        this.routine = routine;
    }
}
