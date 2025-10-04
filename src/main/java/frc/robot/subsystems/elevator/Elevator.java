package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.Mechanism2d;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private final ElevatorIoInputsAutoLogged inputs = new ElevatorIoInputsAutoLogged();
    private final ElevatorIO io;
    private final Mechanism2d mechanism2d;
    private final SysIdRoutine sysId;

    public Elevator(ElevatorIO io, Mechanism2d mechanism2d) {
        this.io = io;
        this.mechanism2d = mechanism2d;
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> io.runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        io.PID();
        io.periodic();
        if (mechanism2d != null)
            mechanism2d.setElevatorLength(Math.max(0.001, getPosition()) + Units.inchesToMeters(31));
    }

    public void setGoal(double goal) {
        io.setGoal(goal);
    }

    public Command setGoal(Distance goal) {
        return Commands.runOnce(() -> io.setGoal(goal.magnitude()), this);
    }

    public double getSetpoint() {
        return io.getSetpoint();
    }

    public double getGoal() {
        return io.getGoal();
    }

    public double getPosition() {
        return io.getPosition();
    }

    public double getLeftVoltage() {
        return io.getLeftVoltage();
    }

    public double getLeftCurrent() {
        return io.getLeftCurrent();
    }

    public double getRightVoltage() {
        return io.getRightVoltage();
    }

    public double getRightCurrent() {
        return io.getRightCurrent();
    }

    public boolean getInPosition() {
        return io.getInPosition();
    }

    public void resetEncoder() {
        io.resetEncoder();
    }

    public void resetTrapezoid() {
        io.resetTrapezoid();
    }

    public SysIdRoutine getSysId() {
        return sysId;
    }
}
