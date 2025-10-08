package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class WristIOSim implements WristIO {
    private final DCMotorSim flippyWristSim;
    private final DCMotorSim spinnyWristSim;

    private final Constraints flippyWristConstraints =
            new Constraints(WristConstants.kFlippyMaxVel, WristConstants.kFlippyMaxAccel);
    private final PIDController flippyWristController =
            new PIDController(WristConstants.kFlippyPSim, WristConstants.kFlippyISim, WristConstants.kFlippyDSim);
    private final ArmFeedforward flippyWristFeedForward =
            new ArmFeedforward(WristConstants.kFlippySSim, WristConstants.kFlippyVSim, WristConstants.kFlippyGSim);
    private TrapezoidProfile.State flippySetpoint;
    private TrapezoidProfile.State flippyGoal;

    private final Constraints spinnyWristConstraints =
            new Constraints(WristConstants.kSpinnyMaxVel, WristConstants.kSpinnyMaxAccel);
    private final PIDController spinnyWristController =
            new PIDController(WristConstants.kSpinnyPSim, WristConstants.kSpinnyISim, WristConstants.kSpinnyDSim);
    private final ArmFeedforward spinnyWristFeedForward =
            new ArmFeedforward(WristConstants.kSpinnySSim, WristConstants.kSpinnyVSim, WristConstants.kSpinnyGSim);
    private TrapezoidProfile.State spinnySetpoint;
    private TrapezoidProfile.State spinnyGoal;

    public WristIOSim() {
        flippyWristSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), .5, WristConstants.kFlippyGearRatio),
                DCMotor.getNEO(1));

        flippyWristSim.setAngle(-Math.PI / 2);

        spinnyWristSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), .2, WristConstants.kSpinnyGearRatio),
                DCMotor.getNeo550(1));

        flippySetpoint = new TrapezoidProfile.State(getFlippyPosition(), 0);
        flippyGoal = flippySetpoint;

        spinnySetpoint = new TrapezoidProfile.State(getSpinnyPosition(), 0);
        spinnyGoal = flippyGoal;
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.flippyGoal = Units.radiansToDegrees(getFlippyGoal());
        inputs.flippyPos = Units.radiansToDegrees(getFlippyPosition());
        inputs.flippySetpoint = Units.radiansToDegrees(getFlippySetpoint());
        inputs.flippyVelocity = Units.radiansToDegrees(getFlippyVelocity());
        inputs.flippyCurrent = getFlippyCurrent();
        inputs.flippyVoltage = getFlippyVoltage();
        inputs.flippyInPosition = getFlippyInPosition();

        inputs.spinnyGoal = Units.radiansToDegrees(getSpinnyGoal());
        inputs.spinnyPos = Units.radiansToDegrees(getSpinnyPosition());
        inputs.spinnySetpoint = Units.radiansToDegrees(getSpinnySetpoint());
        inputs.spinnyVelocity = Units.radiansToDegrees(getSpinnyVelocity());
        inputs.spinnyCurrent = getSpinnyCurrent();
        inputs.spinnyVoltage = getSpinnyVoltage();
        inputs.spinnyInPosition = getSpinnyInPosition();
    }

    @Override
    public double getFlippyPosition() {
        return flippyWristSim.getAngularPositionRotations() * WristConstants.kPositionConversionFactor;
    }

    @Override
    public double getFlippyVoltage() {
        return flippyWristSim.getInputVoltage();
    }

    @Override
    public double getFlippyCurrent() {
        return flippyWristSim.getCurrentDrawAmps();
    }

    @Override
    public void setFlippyGoal(double goal) {
        flippySetpoint = new TrapezoidProfile.State(getFlippyPosition(), 0);
        this.flippyGoal = new TrapezoidProfile.State(goal, 0);
    }

    @Override
    public double getFlippySetpoint() {
        return flippySetpoint.position;
    }

    @Override
    public double getFlippyGoal() {
        return flippyGoal.position;
    }

    @Override
    public double getFlippyVelocity() {
        return flippyWristSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void resetFlippyEncoder() {
        flippyWristSim.setAngle(0);
    }

    @Override
    public void runFlippyCharacterization(double volts) {
        flippyWristSim.setInputVoltage(volts);
    }

    @Override
    public void resetFlippyTrapezoid() {
        flippySetpoint = new TrapezoidProfile.State(getFlippyPosition(), 0);
        flippyGoal = flippySetpoint;
        flippyWristController.setSetpoint(getFlippySetpoint());
    }

    @Override
    public boolean getFlippyInPosition() {
        return Math.abs(getFlippyGoal() - getFlippyPosition()) < WristConstants.kFlippyTolerance;
    }

    @Override
    public double getSpinnyPosition() {
        return spinnyWristSim.getAngularPositionRotations() * WristConstants.kPositionConversionFactor;
    }

    @Override
    public double getSpinnyVoltage() {
        return spinnyWristSim.getInputVoltage();
    }

    @Override
    public double getSpinnyCurrent() {
        return spinnyWristSim.getCurrentDrawAmps();
    }

    @Override
    public void setSpinnyGoal(double goal) {
        spinnySetpoint = new TrapezoidProfile.State(getSpinnyPosition(), 0);
        this.spinnyGoal = new TrapezoidProfile.State(goal, 0);
    }

    @Override
    public double getSpinnySetpoint() {
        return spinnySetpoint.position;
    }

    @Override
    public double getSpinnyGoal() {
        return spinnyGoal.position;
    }

    @Override
    public double getSpinnyVelocity() {
        return spinnyWristSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void resetSpinnyEncoder() {
        spinnyWristSim.setAngle(0);
    }

    @Override
    public void runSpinnyCharacterization(double volts) {
        spinnyWristSim.setInputVoltage(volts);
    }

    @Override
    public void resetSpinnyTrapezoid() {
        spinnySetpoint = new TrapezoidProfile.State(getSpinnyPosition(), 0);
        spinnyGoal = spinnySetpoint;
        spinnyWristController.setSetpoint(getSpinnySetpoint());
    }

    @Override
    public boolean getSpinnyInPosition() {
        return Math.abs(getSpinnyGoal() - getSpinnyPosition()) < WristConstants.kSpinnyTolerance;
    }

    @Override
    public void PID() {
        var flippyProfile = new TrapezoidProfile(flippyWristConstraints).calculate(0.02, flippySetpoint, flippyGoal);
        flippySetpoint = flippyProfile;

        double flippyVoltage = flippyWristController.calculate(getFlippyPosition(), flippyProfile.position)
                + flippyWristFeedForward.calculate(getFlippyPosition(), flippySetpoint.velocity);
        flippyWristSim.setInputVoltage(flippyVoltage);

        var spinnyProfile = new TrapezoidProfile(spinnyWristConstraints).calculate(0.02, spinnySetpoint, spinnyGoal);
        spinnySetpoint = spinnyProfile;

        double spinnyVoltage = spinnyWristController.calculate(getSpinnyPosition(), spinnyProfile.position)
                + spinnyWristFeedForward.calculate(getSpinnyPosition(), spinnySetpoint.velocity);
        spinnyWristSim.setInputVoltage(spinnyVoltage);
    }

    @Override
    public void periodic() {
        flippyWristSim.update(.02);
        spinnyWristSim.update(.02);
    }
}
