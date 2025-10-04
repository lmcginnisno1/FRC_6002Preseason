package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIoInputs {
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

    public default void updateInputs(ElevatorIoInputs inputs) {}

    public default void setGoal(double reference) {}

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

    public default double getVelocity() {
        return 0;
    }

    public default boolean getInPosition() {
        return false;
    }

    public default void resetEncoder() {}

    public default void resetTrapezoid() {}

    public default void runCharacterization(double volts) {}

    public default void PID() {}

    public default void periodic() {}
}
