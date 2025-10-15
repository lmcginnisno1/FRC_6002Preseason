package frc.robot.subsystems.GroundPivot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.Mechanism2d;
import org.littletonrobotics.junction.Logger;

public class GroundPivot extends SubsystemBase {
    private final GroundPivotIO io;
    private final GroundPivotIOInputsAutoLogged inputs = new GroundPivotIOInputsAutoLogged();
    private final Mechanism2d mechanism2d;
    private final SysIdRoutine sysId;

    public GroundPivot(GroundPivotIO io, Mechanism2d mechanism2d) {
        this.io = io;
        this.mechanism2d = mechanism2d;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("GroundPivot/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> io.runCharacterization(voltage.in(Volts)), null, this));
    }

    public void setGoal(double goal) {
        io.setGoal(goal);
    }

    public Command setGoal(Angle goal) {
        return Commands.runOnce(() -> setGoal(goal.magnitude()));
    }

    public double getGoal() {
        return io.getGoal();
    }

    public double getPosition() {
        return io.getPosition();
    }

    public double getVelocity() {
        return io.getVelocity();
    }

    public double getCurrent() {
        return io.getCurrent();
    }

    public double getVoltage() {
        return io.getVoltage();
    }

    public void reset() {
        io.reset();
    }

    public double getSetpoint() {
        return io.getSetpoint();
    }

    public boolean inPosition() {
        return io.inPosition();
    }

    public SysIdRoutine getSysId() {
        return sysId;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("GroundPivot", inputs);

        io.PID();
        io.periodic();

        if (mechanism2d != null) mechanism2d.setGroundPivotAngle(Units.radiansToDegrees(getPosition() + Math.PI/2));
    }
}
