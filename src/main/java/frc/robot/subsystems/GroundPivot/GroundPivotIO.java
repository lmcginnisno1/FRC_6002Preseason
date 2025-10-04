package frc.robot.subsystems.GroundPivot;

import org.littletonrobotics.junction.AutoLog;

public interface GroundPivotIO {
    @AutoLog
    public static class GroundPivotIOInputs {
        public double groundPivotCurrent;
        public double groundPivotVoltage;
        public double groundPivotPosition;
        public double groundPivotGoal;
        public boolean groundPivotInPosition;
        public double groundPivotSetpoint;
        public double groundPivotVelocity;
    }

    public default void updateInputs(GroundPivotIOInputs inputs) {}

    public default void setGoal(double goal) {}

    public default double getGoal() {
        return 0;
    }

    public default double getPosition() {
        return 0;
    }

    public default double getVelocity() {
        return 0;
    }

    public default double getCurrent() {
        return 0;
    }

    public default double getVoltage() {
        return 0;
    }

    public default double getSetpoint() {
        return 0;
    }

    public default boolean inPosition() {
        return false;
    }

    public default void runCharacterization(double volts) {}

    public default void PID() {}

    public default void reset() {}

    public default void periodic() {}
}
