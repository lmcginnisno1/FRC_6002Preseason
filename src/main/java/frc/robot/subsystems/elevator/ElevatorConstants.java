package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    public static final int kLeftElevatorCanId = 3;
    public static final int kRightElevatorCanId = 4;

    public static final boolean kLeftInverted = true;
    public static final boolean kRightInverted = false;

    public static final int kCurrentLimit = 60;
    public static final double gearRatio = 3;
    public static final double drumCircumference = 5.25; // inches

    public static final double kP = 0.4;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;

    public static final double kV = 0.045;
    public static final double kS = 0.25;
    public static final double kG = 0.35;
    public static final double kA = 0.5;

    public static final double kPSim = 3;
    public static final double kISim = 0.0;
    public static final double kDSim = 0.0;

    public static final double kVSim = 0.071167;
    public static final double kSSim = 0.013684;
    public static final double kGSim = 1.4398;
    public static final double kASim = 0.0036608;

    public static final double kMaxVel = Math.toRadians(180);
    public static final double kMaxAccel = Math.toRadians(90);

    public static final double kPositionConversionFactor = 3.135;
    public static final double kVelcoityConversionFactor = kPositionConversionFactor / 60;
    public static final double rotationsToMeters = 1 / ElevatorConstants.gearRatio * .133335;

    public static final double kTolerance = Math.toRadians(3);

    public static final double kHome = 0;
    public static final double kClimb = 13;
    public static final double kReady = 0;
    public static final double kIntake = 0.0;
    public static final double kIntakeGround = 0;
    public static final double kDeployL2 = 7.5;
    public static final double kDeployL3 = 13;
    public static final double kDeployL4 = 66.5;
    public static final double kAlgaeGround = 6;
    public static final double kAlgaeProcessor = 3;
    public static final double kAlgaeCoral = 14.5;
    public static final double kDeployBarge = 78;
    public static final double kReadyIntakeAlgael2 = 9;
    public static final double kReadyIntakeAlgael3 = 23;
}
