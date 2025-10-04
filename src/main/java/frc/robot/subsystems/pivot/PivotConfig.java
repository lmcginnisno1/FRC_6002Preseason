package frc.robot.subsystems.pivot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class PivotConfig {
    public static final SparkMaxConfig leftPivotConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rightPivotConfig = new SparkMaxConfig();

    static {
        leftPivotConfig
                .follow(PivotConstants.kRightPivotCanId)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(PivotConstants.kCurrentLimit)
                .inverted(false)
                .voltageCompensation(12.0);
        leftPivotConfig
                .encoder
                .inverted(PivotConstants.kLeftInverted)
                .positionConversionFactor(PivotConstants.kPositionConversionFactor)
                .velocityConversionFactor(PivotConstants.kVelcoityConversionFactor)
                .quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(10);
        leftPivotConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD, PivotConstants.kFF)
                .outputRange(-1, 1);

        rightPivotConfig
                .disableFollowerMode()
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(PivotConstants.kCurrentLimit)
                .inverted(false)
                .voltageCompensation(12.0);
        rightPivotConfig
                .encoder
                .inverted(false)
                .positionConversionFactor(PivotConstants.kPositionConversionFactor)
                .velocityConversionFactor(PivotConstants.kVelcoityConversionFactor)
                .quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(10);
        rightPivotConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD, PivotConstants.kFF)
                .outputRange(-1, 1);
    }
}
