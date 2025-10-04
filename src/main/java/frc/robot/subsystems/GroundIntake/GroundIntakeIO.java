package frc.robot.subsystems.GroundIntake;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
    @AutoLog
    public static class GroundIntakeIOInputs {
        public double groundIntakeCurrent;
        public double groundIntakeVoltage;
        public double groundIntakeVelocity;
        public double groundIntakeReference;
        public boolean leftPressed;
        public boolean rightPressed;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(GroundIntakeIOInputs inputs) {}

    public default double getCurrent() {
        return 0;
    }

    public default double getVoltage() {
        return 0;
    }

    public default double getReference() {
        return 0;
    }

    public default double getVelocity() {
        return 0;
    }

    public default void setVoltage(double voltage) {}

    public default void setReference(double velocity) {}

    public default boolean hasLeftCoral() {
        return false;
    }

    public default boolean hasRightCoral() {
        return false;
    }

    public default void PID() {}

    public default void periodic() {}
}
