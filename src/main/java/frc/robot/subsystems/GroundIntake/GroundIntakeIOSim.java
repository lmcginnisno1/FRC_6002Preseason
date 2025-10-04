package frc.robot.subsystems.GroundIntake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class GroundIntakeIOSim implements GroundIntakeIO {
    private final DCMotorSim groundIntakeSim;
    private final PIDController groundPIDController =
            new PIDController(GroundIntakeConstants.kSimP, GroundIntakeConstants.kSimI, GroundIntakeConstants.kSimD);
    private double reference = 0;

    public GroundIntakeIOSim() {
        groundIntakeSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), .178, GroundIntakeConstants.kGearRatio),
                DCMotor.getNeo550(1));
    }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        inputs.groundIntakeCurrent = getCurrent();
        inputs.groundIntakeVoltage = getVoltage();
        inputs.groundIntakeReference = getReference();
        inputs.groundIntakeVelocity = Units.radiansToDegrees(getVelocity());
        inputs.leftPressed = false;
        inputs.rightPressed = false;
    }

    @Override
    public void setReference(double reference) {
        this.reference = reference;
    }

    @Override
    public double getVoltage() {
        return groundIntakeSim.getInputVoltage();
    }

    @Override
    public double getCurrent() {
        return groundIntakeSim.getCurrentDrawAmps();
    }

    @Override
    public void PID() {
        groundIntakeSim.setInput(groundPIDController.calculate(reference));
    }

    @Override
    public void periodic() {
        groundIntakeSim.update(0.02);
    }
}
