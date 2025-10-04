package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIoInputs {
        public double pos;
        public double leftCurrent;
        public double leftVoltage;
        public double rightCurrent;
        public double rightVoltage;
        public double setpoint;
        public double goal;
        public double velocity;
        public boolean inPosition;
    }

    public default void updateInputs(PivotIoInputs inputs) {}

    public default void setGoal(double reference) {}

    public default double getVelocity() {
        return 0;
    }

    public default double getGoal() {
        return 0;
    }

    public default double getPosition() {
        return 0;
    }

    public default double getLeftCurrent() {
        return 0;
    }

    public default double getRightCurrent() {
        return 0;
    }

    public default double getSetpoint() {
        return 0;
    }

    public default double getLeftVoltage() {
        return 0;
    }

    public default double getRightVoltage() {
        return 0;
    }

    public default boolean getInPosition() {
        return false;
    }

    public default void resetEncoder() {}

    public default void resetTrapezoid() {}

    public default void periodic() {}

    public default void runCharacterization(double volts) {}

    public default void PID() {}
}
