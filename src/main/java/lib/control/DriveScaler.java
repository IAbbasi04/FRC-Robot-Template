package lib.control;

import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Class that scales the joystick inputs based on scaling type and slew limiting
 */
public class DriveScaler {
    private ScaleType type; // The type of scaling to apply
    private SlewRateLimiter slewLimiter; // Limits acceleration of inputs
    private boolean zeroAtDeadband = false; // The output starts at 0 at the deadband and not at x=0
    private double deadband = 0.03; // The point where the inputs start translating to outputs

    /**
     * Different types of scaling that can be applied to the inputs
     */
    public enum ScaleType {
        LINEAR,
        QUADRATIC,
        CUBIC
    }

    public DriveScaler(ScaleType type, boolean zeroAtDeadband, double deadband) {
        this.type = type;
        this.zeroAtDeadband = zeroAtDeadband;
        this.deadband = deadband;
    }

    public DriveScaler(ScaleType type, boolean zeroAtDeadband) {
        this(type, zeroAtDeadband, 0.03);
    }

    /**
     * Slew limiting is used to limit the rate of change of the joystick
     */
    public DriveScaler withSlewLimit(double slewRate) {
        slewLimiter = new SlewRateLimiter(slewRate);
        return this;
    }

    /**
     * Scales the input by the desired scaling method
     */
    public double scale(double input) {
        double scaledInput = 0.0;

        if (Math.abs(input) > deadband) {
            switch (type) {
                case QUADRATIC:
                case CUBIC:
                case LINEAR:
                    // All polynomial functions follow the same rule
                    if (zeroAtDeadband) {
                        scaledInput = (1 / Math.pow(1 - deadband, type.ordinal() + 1)) * 
                                        Math.pow(input - deadband, type.ordinal() + 1);
                    } else {
                        scaledInput = Math.pow(input, type.ordinal() + 1);
                    }
                    break;
                default:
                    // So far nothing
                    break;
            }
        }

        if (slewLimiter != null) {
            scaledInput = slewLimiter.calculate(scaledInput);
        }
        return scaledInput;
    }
}