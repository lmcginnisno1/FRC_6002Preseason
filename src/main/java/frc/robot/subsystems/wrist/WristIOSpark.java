package frc.robot.subsystems.wrist;

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

public class WristIOSpark implements WristIO {
    private final SparkMax flippyWrist;
    private final SparkMax spinnyWrist;

    private final RelativeEncoder flippyWristEncoder;
    private final SparkClosedLoopController flippyWristController;
    private final ArmFeedforward flippyWristFeedForward =
            new ArmFeedforward(WristConstants.kFlippyS, WristConstants.kFlippyV, WristConstants.kFlippyG);
    private final Constraints flippyWristConstraints =
            new Constraints(WristConstants.kFlippyMaxVel, WristConstants.kFlippyMaxAccel);
    private TrapezoidProfile.State flippySetpoint;
    private TrapezoidProfile.State flippyGoal;

    private final RelativeEncoder spinnyWristEncoder;
    private final SparkClosedLoopController spinnyWristController;
    private final ArmFeedforward spinnyWristFeedForward =
            new ArmFeedforward(WristConstants.kSpinnyS, WristConstants.kSpinnyV, WristConstants.kSpinnyG);
    private final Constraints spinnyWristConstraints =
            new Constraints(WristConstants.kSpinnyMaxVel, WristConstants.kSpinnyMaxAccel);
    private TrapezoidProfile.State spinnySetpoint;
    private TrapezoidProfile.State spinnyGoal;

    private double lastFlippyVel = 0;
    private double lastSpinnyVel = 0;

    public WristIOSpark() {
        flippyWrist = new SparkMax(WristConstants.kFlippyWristCanId, MotorType.kBrushless);
        flippyWristEncoder = flippyWrist.getEncoder();
        flippyWristController = flippyWrist.getClosedLoopController();
        flippyWrist.configure(
                WristConfig.flippyWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        spinnyWrist = new SparkMax(WristConstants.kFlippyWristCanId, MotorType.kBrushless);
        spinnyWristEncoder = spinnyWrist.getEncoder();
        spinnyWristController = spinnyWrist.getClosedLoopController();
        spinnyWrist.configure(
                WristConfig.spinnyWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flippySetpoint = new TrapezoidProfile.State(flippyWristEncoder.getPosition(), 0);
        flippyGoal = flippySetpoint;

        spinnySetpoint = new TrapezoidProfile.State(spinnyWristEncoder.getPosition(), 0);
        spinnyGoal = spinnySetpoint;
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
        return flippyWristEncoder.getPosition();
    }

    @Override
    public double getFlippyVoltage() {
        return flippyWrist.getAppliedOutput() * flippyWrist.getBusVoltage();
    }

    @Override
    public double getFlippyCurrent() {
        return flippyWrist.getOutputCurrent();
    }

    @Override
    public void setFlippyGoal(double goal) {
        flippySetpoint = new TrapezoidProfile.State(getFlippyPosition(), 0);
        setFlippyGoal(goal);
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
        return flippyWristEncoder.getVelocity();
    }

    @Override
    public void resetFlippyEncoder() {
        flippyWristEncoder.setPosition(0);
    }

    @Override
    public void runFlippyCharacterization(double volts) {
        flippyWrist.setVoltage(volts);
    }

    @Override
    public void resetFlippyTrapezoid() {
        flippySetpoint = new TrapezoidProfile.State(getFlippyPosition(), 0);
        flippyGoal = flippySetpoint;
        flippyWristController.setReference(
                flippySetpoint.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                flippyWristFeedForward.calculate(getFlippyPosition(), flippySetpoint.velocity));
    }

    @Override
    public boolean getFlippyInPosition() {
        return Math.abs(getFlippyGoal() - getFlippyPosition()) < WristConstants.kFlippyTolerance;
    }

    @Override
    public double getSpinnyPosition() {
        return spinnyWristEncoder.getPosition();
    }

    @Override
    public double getSpinnyVoltage() {
        return spinnyWrist.getAppliedOutput() * spinnyWrist.getBusVoltage();
    }

    @Override
    public double getSpinnyCurrent() {
        return spinnyWrist.getOutputCurrent();
    }

    @Override
    public void setSpinnyGoal(double goal) {
        spinnySetpoint = new TrapezoidProfile.State(getSpinnyPosition(), 0);
        setSpinnyGoal(goal);
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
        return spinnyWristEncoder.getVelocity();
    }

    @Override
    public void resetSpinnyEncoder() {
        spinnyWristEncoder.setPosition(0);
    }

    @Override
    public void runSpinnyCharacterization(double volts) {
        spinnyWrist.setVoltage(volts);
    }

    @Override
    public void resetSpinnyTrapezoid() {
        spinnySetpoint = new TrapezoidProfile.State(getSpinnyPosition(), 0);
        spinnyGoal = spinnySetpoint;
        spinnyWristController.setReference(
                spinnySetpoint.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                spinnyWristFeedForward.calculate(getSpinnyPosition(), spinnySetpoint.velocity));
    }

    @Override
    public boolean getSpinnyInPosition() {
        return Math.abs(getSpinnyGoal() - getSpinnyPosition()) < WristConstants.kSpinnyTolerance;
    }

    @Override
    public void PID() {
        var flippyProfile = new TrapezoidProfile(flippyWristConstraints).calculate(0.02, flippySetpoint, flippyGoal);
        flippySetpoint = flippyProfile;
        double flippyAccel = (getFlippyVelocity() - lastFlippyVel) / .02;
        flippyWristController.setReference(
                flippySetpoint.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                flippyWristFeedForward.calculate(getFlippyPosition(), flippySetpoint.velocity, flippyAccel));

        var spinnyProfile = new TrapezoidProfile(spinnyWristConstraints).calculate(0.02, spinnySetpoint, spinnyGoal);
        spinnySetpoint = spinnyProfile;
        double spinnyAccel = (getSpinnyVelocity() - lastSpinnyVel) / .02;
        spinnyWristController.setReference(
                spinnySetpoint.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                spinnyWristFeedForward.calculate(getFlippyPosition(), spinnySetpoint.velocity, spinnyAccel));
    }
}
