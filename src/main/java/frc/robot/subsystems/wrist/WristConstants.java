package frc.robot.subsystems.wrist;

import edu.wpi.first.math.util.Units;

public class WristConstants {
    public static final int kFlippyWristCanId = 3;
    public static final int kSpinnyWristCanId = 4;

    public static final boolean kFlippyWristInverted = false;
    public static final boolean kSpinnyWristInverted = false;

    public static final int kCurrentLimit = 60;

    public static final double kFlippyP = 0.0;
    public static final double kFlippyI = 0.0;
    public static final double kFlippyD = 0.0;
    public static final double kFlippyFF = 0.0;

    public static final double kFlippyV = 0.0;
    public static final double kFlippyS = 0.0;
    public static final double kFlippyG = 0.0;
    public static final double kFlippyA = 0.0;

    public static final double kFlippyPSim = 0.0;
    public static final double kFlippyISim = 0.0;
    public static final double kFlippyDSim = 0.0;

    public static final double kFlippyVSim = 0.0;
    public static final double kFlippySSim = 0.0;
    public static final double kFlippyGSim = 0.0;
    public static final double kFlippyASim = 0.0;

    public static final double kFlippyMaxVel = 1;
    public static final double kFlippyMaxAccel = .5;

    public static final double kSpinnyP = 0.0;
    public static final double kSpinnyI = 0.0;
    public static final double kSpinnyD = 0.0;
    public static final double kSpinnyFF = 0.0;

    public static final double kSpinnyV = 0.0;
    public static final double kSpinnyS = 0.0;
    public static final double kSpinnyG = 0.0;
    public static final double kSpinnyA = 0.0;

    public static final double kSpinnyPSim = 6.7341E-15;
    public static final double kSpinnyISim = 0.0;
    public static final double kSpinnyDSim = 0.0;

    public static final double kSpinnyVSim = 0.030015;
    public static final double kSpinnySSim = 0.019395;
    public static final double kSpinnyGSim = 2.325E-05;
    public static final double kSpinnyASim = 4.432E-05;

    public static final double kSpinnyMaxVel = 1;
    public static final double kSpinnyMaxAccel = .5;

    public static final double kPositionConversionFactor = Math.PI * 2;
    public static final double kVelcoityConversionFactor = kPositionConversionFactor / 60;

    public static final double kFlippyGearRatio = 86.67;
    public static final double kSpinnyGearRatio = 216.67;

    public static final double kFlippyTolerance = Units.degreesToRadians(3);
    public static final double kSpinnyTolerance = Units.degreesToRadians(3);

    public static final class FlippyPositions {
        public static final double kHome = Math.toRadians(0);
        public static final double kClimb = Math.toRadians(85);
        public static final double kReady = Math.toRadians(-0);
        public static final double kIntake = Math.toRadians(-26);
        public static final double kIntakeGround = Math.toRadians(-122);
        public static final double kDeployL2 = Math.toRadians(65);
        public static final double kDeployL3 = Math.toRadians(37);
        public static final double kDeployL4 = Math.toRadians(55);
        public static final double kDeployBarge = Math.toRadians(36);
        public static final double kAlgaeProcessor = Math.toRadians(-20);
        public static final double kAlgaeGround = Math.toRadians(-92);
        public static final double kAlgaeCoral = Math.toRadians(-90);
        public static final double kReadyIntakeAlgae = Math.toRadians(100);
    }

    public static final class SpinnyPositions {
        public static final double kIntake = Math.toRadians(-90);
        public static final double kHome = Math.toRadians(-4);
        public static final double kDeployFront = Math.toRadians(-180);
    }
}
