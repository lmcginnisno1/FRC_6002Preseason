package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.Mechanism2d;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private final WristIO io;
    private final Mechanism2d mechanism2d;
    private final SysIdRoutine flippySysId;
    private final SysIdRoutine spinnySysId;

    public Wrist(WristIO io, Mechanism2d mechanism2d) {
        this.io = io;
        this.mechanism2d = mechanism2d;

        flippySysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Wrist/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> io.runFlippyCharacterization(voltage.in(Volts)), null, this));

        spinnySysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Wrist/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> io.runSpinnyCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);

        // io.PID();
        io.periodic();

        if (mechanism2d != null) mechanism2d.setWristAngle(Units.radiansToDegrees(getFlippyPosition()));
    }

    public void setFlippyGoal(double goal) {
        io.setFlippyGoal(goal);
    }

    public Command setFlippyGoal(Angle goal) {
        return Commands.runOnce(() -> io.setFlippyGoal(goal.magnitude()), this);
    }

    public double getFlippySetpoint() {
        return io.getFlippySetpoint();
    }

    public double getFlippyGoal() {
        return io.getFlippyGoal();
    }

    public double getFlippyPosition() {
        return io.getFlippyPosition();
    }

    public double getFlippyVoltage() {
        return io.getFlippyVoltage();
    }

    public double getFlippyCurrent() {
        return io.getFlippyCurrent();
    }

    public boolean getFlippyInPosition() {
        return io.getFlippyInPosition();
    }

    public void resetFlippyEncoder() {
        io.resetFlippyEncoder();
    }

    public void resetFlippyTrapezoid() {
        io.resetFlippyTrapezoid();
    }

    public SysIdRoutine getFlippySysId() {
        return flippySysId;
    }

    public void setSpinnyGoal(double goal) {
        io.setSpinnyGoal(goal);
    }

    public Command setSpinnyGoal(Angle goal) {
        return Commands.runOnce(() -> io.setSpinnyGoal(goal.magnitude()), this);
    }

    public double getSpinnySetpoint() {
        return io.getSpinnySetpoint();
    }

    public double getSpinnyGoal() {
        return io.getSpinnyGoal();
    }

    public double getSpinnyPosition() {
        return io.getSpinnyPosition();
    }

    public double getSpinnyVoltage() {
        return io.getSpinnyVoltage();
    }

    public double getSpinnyCurrent() {
        return io.getSpinnyCurrent();
    }

    public boolean getSpinnyInPosition() {
        return io.getSpinnyInPosition();
    }

    public void resetSpinnyEncoder() {
        io.resetSpinnyEncoder();
    }

    public void resetSpinnyTrapezoid() {
        io.resetSpinnyTrapezoid();
    }

    public SysIdRoutine getSpinnySysId() {
        return spinnySysId;
    }
}
