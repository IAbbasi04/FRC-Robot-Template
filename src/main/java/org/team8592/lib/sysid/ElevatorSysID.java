package org.team8592.lib.sysid;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Deprecated (forRemoval = false)
public class ElevatorSysID extends SysID {
    public ElevatorSysID(Subsystem subsystem) {
        super(
            new SysIdRoutine(
                new SysIdRoutine.Config(), 
                new SysIdRoutine.Mechanism(
                    null, 
                    null, 
                    subsystem
                )
            )
        );
    }
    
    
}