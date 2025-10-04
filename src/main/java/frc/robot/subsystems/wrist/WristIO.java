package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public double flippyPos;
        public double flippyCurrent;
        public double flippyVoltage;
        public double flippySetpoint;
        public double flippyGoal;
        public double flippyVelocity;
        public boolean flippyInPosition;
        public double spinnyPos;
        public double spinnyCurrent;
        public double spinnyVoltage;
        public double spinnySetpoint;
        public double spinnyGoal;
        public double spinnyVelocity;
        public boolean spinnyInPosition;
    }

    public default void updateInputs(WristIOInputs inputs) {}

    public default void setFlippyGoal(double reference) {}

    public default double getFlippyGoal() {
        return 0;
    }

    public default double getFlippyPosition() {
        return 0;
    }

    public default double getFlippySetpoint() {
        return 0;
    }

    public default double getFlippyVoltage() {
        return 0;
    }

    public default double getFlippyCurrent() {
        return 0;
    }

    public default double getFlippyVelocity() {
        return 0;
    }

    public default boolean getFlippyInPosition() {
        return false;
    }

    public default void resetFlippyEncoder() {}

    public default void resetFlippyTrapezoid() {}

    public default void runFlippyCharacterization(double volts) {}

    public default void setSpinnyGoal(double reference) {}

    public default double getSpinnyGoal() {
        return 0;
    }

    public default double getSpinnyPosition() {
        return 0;
    }

    public default double getSpinnySetpoint() {
        return 0;
    }

    public default double getSpinnyVoltage() {
        return 0;
    }

    public default double getSpinnyCurrent() {
        return 0;
    }

    public default double getSpinnyVelocity() {
        return 0;
    }

    public default boolean getSpinnyInPosition() {
        return false;
    }

    public default void resetSpinnyEncoder() {}

    public default void resetSpinnyTrapezoid() {}

    public default void runSpinnyCharacterization(double volts) {}

    public default void PID() {}

    public default void periodic() {}
}
