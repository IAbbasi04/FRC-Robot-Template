package lib.logging;

import java.lang.annotation.*;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
@Deprecated(forRemoval = false)
/**
 * Annotation used to enable or disable a method depending on whether the wrapping subsystem is enabled 
 */
public @interface IfActive {
    public boolean enabled = true;
}