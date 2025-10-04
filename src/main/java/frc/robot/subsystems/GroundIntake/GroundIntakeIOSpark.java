package frc.robot.subsystems.GroundIntake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class GroundIntakeIOSpark implements GroundIntakeIO {
    private final SparkMax groundIntakeMotor;
    private final RelativeEncoder groundIntakeEncoder;
    private final SparkClosedLoopController groundIntakeController;
    private final SparkLimitSwitch leftLimit;
    private final SparkLimitSwitch rightLimit;

    private double groundIntakeReference;
    private ControlType groundIntakeType;

    public GroundIntakeIOSpark() {
        // initialize motor
        groundIntakeMotor = new SparkMax(GroundIntakeConstants.kGroundIntakeCanId, MotorType.kBrushless);

        // initialize PID controller
        groundIntakeController = groundIntakeMotor.getClosedLoopController();

        // initalize encoder
        groundIntakeEncoder = groundIntakeMotor.getEncoder();

        // apply config
        groundIntakeMotor.configure(
                GroundIntakeConfig.groundIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftLimit = groundIntakeMotor.getForwardLimitSwitch();
        rightLimit = groundIntakeMotor.getReverseLimitSwitch();
        // reset target speed in init
        groundIntakeReference = 0;
        groundIntakeType = ControlType.kVoltage;
    }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        inputs.groundIntakeReference = getReference();
        inputs.groundIntakeCurrent = getCurrent();
        inputs.groundIntakeVoltage = getVoltage();
        inputs.groundIntakeVelocity = getVelocity();
        inputs.leftPressed = hasLeftCoral();
        inputs.rightPressed = hasRightCoral();
    }

    @Override
    public double getVelocity() {
        return groundIntakeEncoder.getVelocity();
    }

    @Override
    public double getCurrent() {
        return groundIntakeMotor.getOutputCurrent();
    }

    @Override
    public double getVoltage() {
        return groundIntakeMotor.getBusVoltage() * groundIntakeMotor.getAppliedOutput();
    }

    @Override
    public double getReference() {
        return groundIntakeReference;
    }

    @Override
    public void setVoltage(double voltage) {
        groundIntakeReference = voltage;
        groundIntakeType = ControlType.kVoltage;
    }

    @Override
    public void setReference(double velocity) {
        groundIntakeReference = velocity;
        groundIntakeType = ControlType.kVelocity;
    }

    @Override
    public boolean hasLeftCoral() {
        return leftLimit.isPressed();
    }

    @Override
    public boolean hasRightCoral() {
        return rightLimit.isPressed();
    }

    @Override
    public void PID() {
        groundIntakeController.setReference(groundIntakeReference, groundIntakeType);
    }
}
