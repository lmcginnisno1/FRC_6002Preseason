package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorConfig {
    public static final SparkMaxConfig leftElevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rightElevatorConfig = new SparkMaxConfig();

    static {
        leftElevatorConfig
                .follow(ElevatorConstants.kRightElevatorCanId)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ElevatorConstants.kCurrentLimit)
                .inverted(false)
                .voltageCompensation(12.0);
        leftElevatorConfig
                .encoder
                .inverted(ElevatorConstants.kLeftInverted)
                .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
                .velocityConversionFactor(ElevatorConstants.kVelcoityConversionFactor)
                .quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(10);
        leftElevatorConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kFF)
                .outputRange(-1, 1);

        rightElevatorConfig
                .disableFollowerMode()
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ElevatorConstants.kCurrentLimit)
                .inverted(false)
                .voltageCompensation(12.0);
        rightElevatorConfig
                .encoder
                .inverted(false)
                .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
                .velocityConversionFactor(ElevatorConstants.kVelcoityConversionFactor)
                .quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(10);
        rightElevatorConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kFF)
                .outputRange(-1, 1);
    }
}
