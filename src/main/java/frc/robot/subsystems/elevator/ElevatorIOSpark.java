package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ElevatorIOSpark implements ElevatorIO {
    private final SparkMax leftElevator;
    private final SparkMax rightElevator;
    private final RelativeEncoder elevatorEncoder;
    private final SparkClosedLoopController elevatorController;
    private final ElevatorFeedforward elevatorFeedforward =
            new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kV, ElevatorConstants.kG);
    private final Constraints elevatorConstraints =
            new Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAccel);
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;

    public ElevatorIOSpark() {
        leftElevator = new SparkMax(ElevatorConstants.kLeftElevatorCanId, MotorType.kBrushless);
        leftElevator.configure(
                ElevatorConfig.leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightElevator = new SparkMax(ElevatorConstants.kRightElevatorCanId, MotorType.kBrushless);
        elevatorEncoder = rightElevator.getEncoder();
        elevatorController = rightElevator.getClosedLoopController();
        rightElevator.configure(
                ElevatorConfig.rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setpoint = new TrapezoidProfile.State(elevatorEncoder.getPosition(), 0);
        goal = setpoint;
    }

    @Override
    public void updateInputs(ElevatorIoInputs inputs) {
        inputs.goal = getGoal();
        inputs.pos = getPosition();
        inputs.setpoint = getSetpoint();
        inputs.velocity = getVelocity();
        inputs.leftCurrent = getLeftCurrent();
        inputs.leftVoltage = getLeftVoltage();
        inputs.rightCurrent = getRightCurrent();
        inputs.rightVoltage = getRightVoltage();
        inputs.inPosition = getInPosition();
    }

    @Override
    public double getPosition() {
        return elevatorEncoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return elevatorEncoder.getVelocity();
    }

    @Override
    public double getLeftVoltage() {
        return leftElevator.getAppliedOutput() * leftElevator.getBusVoltage();
    }

    @Override
    public double getLeftCurrent() {
        return leftElevator.getOutputCurrent();
    }

    @Override
    public double getRightVoltage() {
        return rightElevator.getAppliedOutput() * rightElevator.getBusVoltage();
    }

    @Override
    public double getRightCurrent() {
        return rightElevator.getOutputCurrent();
    }

    @Override
    public void setGoal(double goal) {
        setpoint = new TrapezoidProfile.State(getPosition(), 0);
        setGoal(goal);
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
    public void resetEncoder() {
        elevatorEncoder.setPosition(0);
    }

    @Override
    public void runCharacterization(double volts) {
        leftElevator.setVoltage(volts);
        rightElevator.setVoltage(volts);
    }

    @Override
    public void resetTrapezoid() {
        setpoint = new TrapezoidProfile.State(getPosition(), 0);
        goal = setpoint;
        elevatorController.setReference(
                setpoint.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                elevatorFeedforward.calculate(setpoint.velocity));
    }

    @Override
    public boolean getInPosition() {
        return Math.abs(getGoal() - getPosition()) < ElevatorConstants.kTolerance;
    }

    @Override
    public void PID() {
        var profile = new TrapezoidProfile(elevatorConstraints).calculate(0.02, setpoint, goal);
        setpoint = profile;
        elevatorController.setReference(
                setpoint.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                elevatorFeedforward.calculate(setpoint.velocity));
    }

    @Override
    public void periodic() {}
}
