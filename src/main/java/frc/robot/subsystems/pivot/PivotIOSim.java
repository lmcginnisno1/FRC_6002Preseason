package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotIOSim implements PivotIO {
    private final DCMotorSim pivotSim;
    private final Constraints pivotConstraints = new Constraints(PivotConstants.kMaxVel, PivotConstants.kMaxAccel);
    private final PIDController pivotController =
            new PIDController(PivotConstants.kPSim, PivotConstants.kISim, PivotConstants.kDSim);
    private final ArmFeedforward pivotFeedforward =
            new ArmFeedforward(PivotConstants.kSSim, PivotConstants.kGSim, PivotConstants.kVSim, PivotConstants.kA);
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;
    private double lastVel = 0;
    private double currentVel = 0;

    public PivotIOSim() {
        pivotSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNEO(2), 4.68, PivotConstants.gearRatio),
                DCMotor.getNEO(2));

        pivotSim.setAngle(PivotConstants.kHome);

        setpoint = new TrapezoidProfile.State(Units.degreesToRadians(0), 0);
        goal = setpoint;
    }

    @Override
    public void updateInputs(PivotIoInputs inputs) {
        inputs.goal = Units.radiansToDegrees(getGoal());
        inputs.pos = Units.radiansToDegrees(getPosition());
        inputs.setpoint = Units.radiansToDegrees(getSetpoint());
        inputs.velocity = getVelocity();
        inputs.leftCurrent = getLeftCurrent();
        inputs.leftVoltage = getLeftVoltage();
        inputs.rightCurrent = getRightCurrent();
        inputs.rightVoltage = getRightVoltage();
        inputs.inPosition = getInPosition();
    }

    @Override
    public double getPosition() {
        return pivotSim.getAngularPositionRad();
    }

    @Override
    public double getLeftVoltage() {
        return pivotSim.getInputVoltage();
    }

    @Override
    public double getLeftCurrent() {
        return pivotSim.getCurrentDrawAmps();
    }

    @Override
    public double getRightVoltage() {
        return pivotSim.getInputVoltage();
    }

    @Override
    public double getRightCurrent() {
        return pivotSim.getCurrentDrawAmps();
    }

    @Override
    public void setGoal(double goal) {
        setpoint = new TrapezoidProfile.State(getPosition(), 0);
        this.goal = new TrapezoidProfile.State(goal, 0);
    }

    @Override
    public double getSetpoint() {
        return setpoint.position;
    }

    @Override
    public double getGoal() {
        return goal.position;
    }

    @Override
    public double getVelocity() {
        return pivotSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void resetEncoder() {
        pivotSim.setAngle(0);
    }

    @Override
    public void resetTrapezoid() {
        setpoint = new TrapezoidProfile.State(getPosition(), 0);
        goal = setpoint;
        pivotController.setSetpoint(getSetpoint());
    }

    @Override
    public boolean getInPosition() {
        return Math.abs(getGoal() - getPosition()) < PivotConstants.kTolerance;
    }

    @Override
    public void runCharacterization(double volts) {
        pivotSim.setInputVoltage(volts);
    }

    @Override
    public void PID() {
        var profile = new TrapezoidProfile(pivotConstraints).calculate(0.02, setpoint, goal);
        setpoint = profile;

        currentVel = getVelocity();
        double accel = (currentVel - lastVel) / .02;

        double voltage = pivotController.calculate(getPosition(), profile.position)
                + pivotFeedforward.calculate(getPosition(), setpoint.velocity, accel);

        pivotSim.setInputVoltage(voltage);

        lastVel = getVelocity();
    }

    @Override
    public void periodic() {
        pivotSim.update(.02);
    }
}
