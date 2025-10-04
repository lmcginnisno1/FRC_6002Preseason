package frc.robot.subsystems.pivot;

import edu.wpi.first.math.util.Units;

public class PivotConstants {
    public static final int kLeftPivotCanId = 3;
    public static final int kRightPivotCanId = 4;

    public static final boolean kLeftInverted = true;
    public static final boolean kRightInverted = false;

    public static final int kCurrentLimit = 60;
    public static final double gearRatio = 216.6666666666667;

    public static final double kP = 2;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;

    public static final double kV = 3.0;
    public static final double kS = 0.025;
    public static final double kG = 0.025;
    public static final double kA = 0.0;

    public static final double kPSim = 0.002828;
    public static final double kISim = 0.0;
    public static final double kDSim = 0.0;

    public static final double kVSim = 4.2923;
    public static final double kSSim = 0.022051;
    public static final double kGSim = 0.0;
    public static final double kASim = 0.031434;

    public static final double kMaxVel = Units.degreesToRadians(30);
    public static final double kMaxAccel = Units.degreesToRadians(30);

    public static final double kPositionConversionFactor = Math.PI * 2;
    public static final double kVelcoityConversionFactor = kPositionConversionFactor / 60;

    public static final double kPositionConversionFactorSim = Math.PI * 2 * gearRatio;
    public static final double kVelcoityConversionFactorSim = kPositionConversionFactor / 60;

    public static final double kTolerance = 3;

    public static final double kHome = Math.toRadians(60);
    public static final double kReady = Math.toRadians(64);
    public static final double kIntake = Math.toRadians(61);
    public static final double kIntakeGround = Math.toRadians(67);
    public static final double kDeployL2 = Math.toRadians(88);
    public static final double kDeployL3 = Math.toRadians(90);
    public static final double kDeployL4 = Math.toRadians(85);
    public static final double kDeployBarge = Math.toRadians(90);
    public static final double kReadyIntakeAlgae = Math.toRadians(100);
    public static final double kReadyIntakeAlgael3 = Math.toRadians(90);
    public static final double kAlgaeProcessor = Math.toRadians(18);
    public static final double kAlgaeGround = Math.toRadians(30);
    public static final double kAlgaeCoral = Math.toRadians(65);
    public static final double kClimb = Math.toRadians(95);
}
