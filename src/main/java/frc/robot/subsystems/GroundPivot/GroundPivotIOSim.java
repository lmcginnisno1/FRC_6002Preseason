package frc.robot.subsystems.GroundPivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class GroundPivotIOSim implements GroundPivotIO {
    private final SingleJointedArmSim groundPivotSim;
    private final Constraints groundPivotConstraints =
            new Constraints(GroundPivotConstants.kMaxVel, GroundPivotConstants.kMaxAccel);
    private final PIDController groundPivotController =
            new PIDController(GroundPivotConstants.kPSim, GroundPivotConstants.kISim, GroundPivotConstants.kDSim);
    private final ArmFeedforward groundPivotFeedforward = new ArmFeedforward(
            GroundPivotConstants.kSSim,
            GroundPivotConstants.kGSim,
            GroundPivotConstants.kVSim,
            GroundPivotConstants.kASim);
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;
    private double lastVel = 0;
    private double currentVel = 0;
    private double groundPivotVoltage = 0;

    public GroundPivotIOSim() {
        groundPivotSim = new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), .3, GroundPivotConstants.kGearRatio),
            DCMotor.getNEO(1),
            GroundPivotConstants.kGearRatio,
            Units.inchesToMeters(13.5),
            -Math.PI/2,
            Math.PI/2,
            true,
            -Math.PI/2
        );

        groundPivotSim.setState(-Math.PI / 2, 0);

        setpoint = new TrapezoidProfile.State(getPosition(), 0);
        goal = setpoint;
    }

    @Override
    public void updateInputs(GroundPivotIOInputs inputs) {
        inputs.groundPivotCurrent = getCurrent();
        inputs.groundPivotVoltage = getVoltage();
        inputs.groundPivotGoal = Units.radiansToDegrees(goal.position);
        inputs.groundPivotSetpoint = Units.radiansToDegrees(setpoint.position);
        inputs.groundPivotPosition = Units.radiansToDegrees(getPosition());
        inputs.groundPivotVelocity = Units.radiansToDegrees(getVelocity());
        inputs.groundPivotInPosition = inPosition();
    }

    @Override
    public double getPosition() {
        return groundPivotSim.getAngleRads();
    }

    @Override
    public double getVelocity() {
        return groundPivotSim.getVelocityRadPerSec();
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
    public void setGoal(double goal) {
        setpoint = new TrapezoidProfile.State(getPosition(), 0);
        this.goal = new TrapezoidProfile.State(goal, 0);
    }

    @Override
    public double getVoltage() {
        return groundPivotVoltage;
    }

    @Override
    public double getCurrent() {
        return groundPivotSim.getCurrentDrawAmps();
    }

    @Override
    public boolean inPosition() {
        return Math.abs(goal.position - setpoint.position) < GroundPivotConstants.kTolerance;
    }

    @Override
    public void reset() {
        setpoint = new TrapezoidProfile.State(getPosition(), 0);
        goal = setpoint;
        groundPivotController.setSetpoint(getSetpoint());
    }

    @Override
    public void runCharacterization(double volts) {
        groundPivotSim.setInput(volts);
        groundPivotVoltage = volts;
    }

    @Override
    public void PID() {
        var profile = new TrapezoidProfile(groundPivotConstraints).calculate(0.02, setpoint, goal);
        setpoint = profile;

        currentVel = getVelocity();
        double accel = (currentVel - lastVel) / .02;

        double voltage = groundPivotController.calculate(getPosition(), profile.position)
                + groundPivotFeedforward.calculate(getPosition(), setpoint.velocity, accel);

        groundPivotSim.setInput(voltage);
        groundPivotVoltage = voltage;

        lastVel = getVelocity();
    }

    @Override
    public void periodic() {
        groundPivotSim.update(0.02);
    }
}
