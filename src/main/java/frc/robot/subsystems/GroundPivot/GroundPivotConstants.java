package frc.robot.subsystems.GroundPivot;

public class GroundPivotConstants {
    public static final int kGroundPivotCanId = 12;
    public static final boolean kInverted = false;
    public static final double kGearRatio = 72;

    public static final double kP = 0.2;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double kV = 1.1;
    public static final double kG = 0.15;
    public static final double kS = 0.0;
    public static final double kA = .14;

    public static final double kPSim = 0.0039071;
    public static final double kISim = 0.0;
    public static final double kDSim = 0.0;
    public static final double kFFSim = 0.0;
    public static final double kVSim = 0.024453;
    public static final double kGSim = 1.8296;
    public static final double kSSim = 0.037411;
    public static final double kASim = 0.00079128;

    public static final double kConversionFactor = 2 * Math.PI;
    public static final double kOffset = Math.toRadians(-180);
    public static final double kMaxVel = Math.toRadians(1000);
    public static final double kMaxAccel = Math.toRadians(1000);
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;
    public static final double kTolerance = Math.toRadians(3);

    public static final double kHome = Math.toRadians(-10);
    public static final double kTransfer = Math.toRadians(-18);
    public static final double kClimb = Math.toRadians(112);
    public static final double kClimbUp = Math.toRadians(80);
    public static final double kProcessor = Math.toRadians(-80);
    public static final double kAlgaeGround = Math.toRadians(-80);
    public static final double kIntake = Math.toRadians(112);
    public static final double kDeploy = Math.toRadians(10);
}
