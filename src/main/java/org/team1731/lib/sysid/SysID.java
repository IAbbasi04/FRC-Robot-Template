package org.team1731.lib.sysid;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Deprecated (forRemoval = false)
public abstract class SysID {
    protected SysIdRoutine routine;
    public SysID(SysIdRoutine routine) {
        this.routine = routine;
    }
}
