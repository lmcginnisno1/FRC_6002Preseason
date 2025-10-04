package frc.robot.subsystems.pivot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class PivotIOSpark implements PivotIO {
    private final SparkMax leftPivot;
    private final SparkMax rightPivot;
    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotController;
    private final ArmFeedforward pivotFeedforward =
            new ArmFeedforward(PivotConstants.kS, PivotConstants.kV, PivotConstants.kG);
    private final Constraints pivotConstraints = new Constraints(PivotConstants.kMaxVel, PivotConstants.kMaxAccel);
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;

    public PivotIOSpark() {
        leftPivot = new SparkMax(PivotConstants.kLeftPivotCanId, MotorType.kBrushless);
        leftPivot.configure(
                PivotConfig.leftPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightPivot = new SparkMax(PivotConstants.kRightPivotCanId, MotorType.kBrushless);
        pivotEncoder = rightPivot.getEncoder();
        pivotController = rightPivot.getClosedLoopController();
        rightPivot.configure(
                PivotConfig.rightPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setpoint = new TrapezoidProfile.State(pivotEncoder.getPosition(), 0);
        goal = setpoint;
    }

    @Override
    public void updateInputs(PivotIoInputs inputs) {
        inputs.goal = Units.radiansToDegrees(getGoal());
        inputs.pos = Units.radiansToDegrees(getPosition());
        inputs.setpoint = Units.radiansToDegrees(getSetpoint());
        inputs.velocity = Units.radiansToDegrees(getVelocity());
        inputs.leftCurrent = getLeftCurrent();
        inputs.leftVoltage = getLeftVoltage();
        inputs.rightCurrent = getRightCurrent();
        inputs.rightVoltage = getRightVoltage();
        inputs.inPosition = getInPosition();
    }

    @Override
    public double getPosition() {
        return pivotEncoder.getPosition();
    }

    @Override
    public double getLeftVoltage() {
        return leftPivot.getAppliedOutput() * leftPivot.getBusVoltage();
    }

    @Override
    public double getLeftCurrent() {
        return leftPivot.getOutputCurrent();
    }

    @Override
    public double getRightVoltage() {
        return rightPivot.getAppliedOutput() * rightPivot.getBusVoltage();
    }

    @Override
    public double getRightCurrent() {
        return rightPivot.getOutputCurrent();
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
    public double getVelocity() {
        return pivotEncoder.getVelocity();
    }

    @Override
    public void resetEncoder() {
        pivotEncoder.setPosition(0);
    }

    @Override
    public void runCharacterization(double volts) {
        rightPivot.setVoltage(volts);
        leftPivot.setVoltage(volts);
    }

    @Override
    public void resetTrapezoid() {
        setpoint = new TrapezoidProfile.State(getPosition(), 0);
        goal = setpoint;
        pivotController.setReference(
                setpoint.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                pivotFeedforward.calculate(getPosition(), setpoint.velocity));
    }

    @Override
    public boolean getInPosition() {
        return Math.abs(getGoal() - getPosition()) < PivotConstants.kTolerance;
    }

    @Override
    public void PID() {
        var profile = new TrapezoidProfile(pivotConstraints).calculate(0.02, setpoint, goal);
        setpoint = profile;
        pivotController.setReference(
                setpoint.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                pivotFeedforward.calculate(getPosition(), setpoint.velocity));
    }

    @Override
    public void periodic() {}
}
