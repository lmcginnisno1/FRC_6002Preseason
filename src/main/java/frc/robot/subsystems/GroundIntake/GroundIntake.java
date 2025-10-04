package frc.robot.subsystems.GroundIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class GroundIntake extends SubsystemBase {
    private final GroundIntakeIO io;
    private final GroundIntakeIOInputsAutoLogged inputs = new GroundIntakeIOInputsAutoLogged();

    public GroundIntake(GroundIntakeIO io) {
        this.io = io;
    }

    public double getReference() {
        return io.getReference();
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

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void setReference(double velocity) {
        io.setReference(velocity);
    }

    public boolean hasCorral() {
        return io.hasLeftCoral() && io.hasRightCoral();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.PID();
        io.periodic();
        Logger.processInputs("GroundIntake", inputs);

        SmartDashboard.putNumber("intake speed", getVelocity());
        SmartDashboard.putNumber("intake goal", getReference());
        SmartDashboard.putNumber("intake current", getCurrent());
    }
}
