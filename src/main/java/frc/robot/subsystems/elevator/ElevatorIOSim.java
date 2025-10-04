package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim elevatorSim;
    private final Constraints elevatorConstraints =
            new Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAccel);
    private final PIDController elevatorController =
            new PIDController(ElevatorConstants.kPSim, ElevatorConstants.kISim, ElevatorConstants.kDSim);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(
            ElevatorConstants.kSSim, ElevatorConstants.kGSim, ElevatorConstants.kVSim, ElevatorConstants.kASim);
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;
    private double inputVoltage = 0;
    private double lastVel = 0;

    public ElevatorIOSim() {
        elevatorSim = new ElevatorSim(
                DCMotor.getNEO(2), ElevatorConstants.gearRatio, 9.0, .02122, 0, Units.inchesToMeters(80), true, 0);

        setpoint = new TrapezoidProfile.State(getPosition(), 0);
        goal = setpoint;
    }

    @Override
    public void updateInputs(ElevatorIoInputs inputs) {
        inputs.goal = Units.metersToInches(getGoal());
        inputs.pos = Units.metersToInches(getPosition());
        inputs.setpoint = Units.metersToInches(getSetpoint());
        inputs.velocity = Units.metersToInches(getVelocity());
        inputs.leftCurrent = getLeftCurrent();
        inputs.leftVoltage = getLeftVoltage();
        inputs.rightCurrent = getRightCurrent();
        inputs.rightVoltage = getRightVoltage();
        inputs.inPosition = getInPosition();
    }

    @Override
    public double getPosition() {
        return elevatorSim.getPositionMeters();
    }

    @Override
    public double getLeftVoltage() {
        return inputVoltage;
    }

    @Override
    public double getLeftCurrent() {
        return elevatorSim.getCurrentDrawAmps();
    }

    @Override
    public double getRightVoltage() {
        return inputVoltage;
    }

    @Override
    public double getRightCurrent() {
        return elevatorSim.getCurrentDrawAmps();
    }

    @Override
    public void setGoal(double goal) {
        setpoint = new TrapezoidProfile.State(getPosition(), 0);
        this.goal = new TrapezoidProfile.State(Units.inchesToMeters(goal), 0);
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
        return elevatorSim.getVelocityMetersPerSecond();
    }

    @Override
    public void resetEncoder() {
        elevatorSim.setState(Units.inchesToMeters(.01), 0);
    }

    @Override
    public void runCharacterization(double volts) {
        elevatorSim.setInputVoltage(volts);
        inputVoltage = volts;
    }

    @Override
    public void resetTrapezoid() {
        setpoint = new TrapezoidProfile.State(getPosition(), 0);
        goal = setpoint;
        elevatorController.setSetpoint(getSetpoint());
    }

    @Override
    public boolean getInPosition() {
        return Math.abs(getGoal() - getPosition()) < ElevatorConstants.kTolerance;
    }

    @Override
    public void PID() {
        var profile = new TrapezoidProfile(elevatorConstraints).calculate(0.02, setpoint, goal);
        setpoint = profile;

        double voltage = elevatorController.calculate(getPosition(), profile.position)
                + elevatorFeedforward.calculate(setpoint.velocity, (getVelocity() - lastVel) / 0.02);

        elevatorSim.setInputVoltage(voltage);

        inputVoltage = voltage;
        lastVel = getVelocity();
    }

    @Override
    public void periodic() {
        elevatorSim.update(.02);
    }
}
