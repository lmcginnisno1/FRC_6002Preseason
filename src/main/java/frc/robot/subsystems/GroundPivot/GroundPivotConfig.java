package frc.robot.subsystems.GroundPivot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class GroundPivotConfig {
    public static final SparkMaxConfig groundPivotConfig = new SparkMaxConfig();

    static {
        groundPivotConfig
                .idleMode(IdleMode.kBrake)
                .inverted(GroundPivotConstants.kInverted)
                .smartCurrentLimit(40)
                .disableFollowerMode()
                .voltageCompensation(12.0);
        groundPivotConfig
                .absoluteEncoder
                .inverted(true)
                .positionConversionFactor(GroundPivotConstants.kConversionFactor)
                .velocityConversionFactor(GroundPivotConstants.kConversionFactor / 60)
                .averageDepth(2);
        groundPivotConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .outputRange(GroundPivotConstants.kMinOutput, GroundPivotConstants.kMaxOutput)
                .pidf(
                        GroundPivotConstants.kP,
                        GroundPivotConstants.kI,
                        GroundPivotConstants.kD,
                        GroundPivotConstants.kFF);
        groundPivotConfig.limitSwitch.forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false);
    }
}
