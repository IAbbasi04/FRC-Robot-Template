package lib.team8592.logging;

import java.lang.annotation.*;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
@Deprecated(forRemoval = false)
public @interface IfActive {
    public boolean enabled = true;
}