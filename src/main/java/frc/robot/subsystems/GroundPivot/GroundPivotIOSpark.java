package frc.robot.subsystems.GroundPivot;

import com.revrobotics.AbsoluteEncoder;
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

public class GroundPivotIOSpark implements GroundPivotIO {
    private final SparkMax groundPivotMotor;
    private final AbsoluteEncoder groundPivotEncoder;
    private final SparkClosedLoopController groundPivotController;
    private ArmFeedforward groundPivotFeedforward = new ArmFeedforward(
            GroundPivotConstants.kS, GroundPivotConstants.kG, GroundPivotConstants.kV, GroundPivotConstants.kA);
    private Constraints groundPivotFeedConstraints =
            new Constraints(GroundPivotConstants.kMaxVel, GroundPivotConstants.kMaxAccel);
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    public GroundPivotIOSpark() {
        // initialize motor
        groundPivotMotor = new SparkMax(GroundPivotConstants.kGroundPivotCanId, MotorType.kBrushless);

        // initialize PID controller
        groundPivotController = groundPivotMotor.getClosedLoopController();

        // initalize encoder
        groundPivotEncoder = groundPivotMotor.getAbsoluteEncoder();

        // apply config
        groundPivotMotor.configure(
                GroundPivotConfig.groundPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // reset reference in init
        setpoint = new TrapezoidProfile.State(getPosition() - GroundPivotConstants.kOffset, 0);
        goal = setpoint;
    }

    @Override
    public void updateInputs(GroundPivotIOInputs inputs) {
        inputs.groundPivotGoal = Math.toDegrees(getGoal());
        inputs.groundPivotSetpoint = Math.toDegrees(getSetpoint());
        inputs.groundPivotPosition = Math.toDegrees(getPosition());
        inputs.groundPivotVelocity = Math.toDegrees(getVelocity());
        inputs.groundPivotCurrent = getCurrent();
        inputs.groundPivotVoltage = getVoltage();
        inputs.groundPivotInPosition = inPosition();
    }

    @Override
    public void setGoal(double goal) {
        setpoint = new TrapezoidProfile.State(getPosition() - GroundPivotConstants.kOffset, 0);
        this.goal = new TrapezoidProfile.State(goal - GroundPivotConstants.kOffset, 0);
    }

    @Override
    public double getPosition() {
        return groundPivotEncoder.getPosition() + GroundPivotConstants.kOffset;
    }

    @Override
    public double getVelocity() {
        return groundPivotEncoder.getVelocity();
    }

    @Override
    public double getCurrent() {
        return groundPivotMotor.getOutputCurrent();
    }

    @Override
    public double getVoltage() {
        return groundPivotMotor.getBusVoltage() * groundPivotMotor.getAppliedOutput();
    }

    @Override
    public double getGoal() {
        return goal.position + GroundPivotConstants.kOffset;
    }

    @Override
    public double getSetpoint() {
        return setpoint.position + GroundPivotConstants.kOffset;
    }

    @Override
    public boolean inPosition() {
        return Math.abs(getPosition() - getGoal()) < GroundPivotConstants.kTolerance;
    }

    @Override
    public void runCharacterization(double volts) {
        groundPivotMotor.setVoltage(volts);
    }

    @Override
    public void PID() {
        double lastSetpoint = setpoint.position;

        var profile = new TrapezoidProfile(groundPivotFeedConstraints).calculate(0.02, setpoint, goal);
        setpoint = profile;

        double acceleration = (setpoint.position - lastSetpoint) / 0.02;

        groundPivotController.setReference(
                setpoint.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                groundPivotFeedforward.calculate(
                        getPosition() + GroundPivotConstants.kOffset - Math.toRadians(90),
                        setpoint.velocity,
                        acceleration));
    }

    @Override
    public void reset() {
        setpoint = new TrapezoidProfile.State(getPosition() - GroundPivotConstants.kOffset, 0);
        goal = setpoint;
        groundPivotController.setReference(
                setpoint.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                groundPivotFeedforward.calculate(
                        getPosition() + GroundPivotConstants.kOffset - Math.toRadians(90), setpoint.velocity));
    }
}
