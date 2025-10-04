package frc.robot.subsystems.GroundIntake;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class GroundIntakeConfig {
    public static final SparkMaxConfig groundIntakeConfig = new SparkMaxConfig();
    public static final LimitSwitchConfig groundIntakeLimitSwitchConfig = new LimitSwitchConfig();

    static {
        groundIntakeLimitSwitchConfig
                .setSparkMaxDataPortConfig()
                .reverseLimitSwitchType(com.revrobotics.spark.config.LimitSwitchConfig.Type.kNormallyOpen)
                .forwardLimitSwitchType(com.revrobotics.spark.config.LimitSwitchConfig.Type.kNormallyOpen);

        groundIntakeConfig
                .disableFollowerMode()
                .idleMode(IdleMode.kBrake)
                .inverted(GroundIntakeConstants.kInverted)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0)
                .limitSwitch
                .apply(groundIntakeLimitSwitchConfig);
        groundIntakeConfig.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(10);
        groundIntakeConfig
                .closedLoop
                .pidf(
                        GroundIntakeConstants.kP,
                        GroundIntakeConstants.kI,
                        GroundIntakeConstants.kD,
                        GroundIntakeConstants.kFF)
                .outputRange(GroundIntakeConstants.kMinOutput, GroundIntakeConstants.kMaxOutput)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        groundIntakeConfig.limitSwitch.forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false);
    }
}
