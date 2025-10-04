package frc.robot.subsystems.wrist;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class WristConfig {
    public static final SparkMaxConfig flippyWristConfig = new SparkMaxConfig();
    public static final SparkMaxConfig spinnyWristConfig = new SparkMaxConfig();

    static {
        flippyWristConfig
                .disableFollowerMode()
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(WristConstants.kCurrentLimit)
                .inverted(WristConstants.kFlippyWristInverted)
                .voltageCompensation(12.0);
        flippyWristConfig
                .absoluteEncoder
                .inverted(false)
                .positionConversionFactor(WristConstants.kPositionConversionFactor)
                .velocityConversionFactor(WristConstants.kVelcoityConversionFactor)
                .averageDepth(2);
        flippyWristConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(
                        WristConstants.kFlippyP,
                        WristConstants.kFlippyI,
                        WristConstants.kFlippyD,
                        WristConstants.kFlippyFF)
                .outputRange(-1, 1);

        spinnyWristConfig
                .disableFollowerMode()
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(WristConstants.kCurrentLimit)
                .inverted(WristConstants.kSpinnyWristInverted)
                .voltageCompensation(12.0);
        spinnyWristConfig
                .absoluteEncoder
                .inverted(false)
                .positionConversionFactor(WristConstants.kPositionConversionFactor)
                .velocityConversionFactor(WristConstants.kVelcoityConversionFactor)
                .averageDepth(2);
        spinnyWristConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(
                        WristConstants.kFlippyP,
                        WristConstants.kFlippyI,
                        WristConstants.kFlippyD,
                        WristConstants.kFlippyFF)
                .outputRange(-1, 1);
    }
}
