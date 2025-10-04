package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double odometryFrequency = 50.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(22.5);
    public static final double wheelBase = Units.inchesToMeters(23.5);
    public static final double bumperWidth = 32;
    public static final double bumperLength = 32.5;
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

    // Device CAN IDs
    public static final int frontLeftDriveCanId = 2;
    public static final int backLeftDriveCanId = 19;
    public static final int frontRightDriveCanId = 8;
    public static final int backRightDriveCanId = 10;

    public static final int frontLeftTurnCanId = 1;
    public static final int backLeftTurnCanId = 5;
    public static final int frontRightTurnCanId = 9;
    public static final int backRightTurnCanId = 7;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 80;
    public static final double wheelRadiusMeters = .0762;
    public static final double driveMotorReduction =
            (45.0 * 22.0) / (12.0 * 15.0); // MAXSwerve with 12 pinion teeth and 22 spur teeth
    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
            2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
            (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration

    public static final double driveKp = 0.012;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.12796;
    public static final double driveKv = 0.093882;
    public static final double driveKa = 0.004;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 2.0;
    public static final double turnKs = 0.0;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = 50;
    public static final double robotMOI = 5;
    public static final double wheelCOF = .7;
    public static final RobotConfig ppConfig = new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                    wheelRadiusMeters,
                    maxSpeedMetersPerSec,
                    wheelCOF,
                    driveGearbox.withReduction(driveMotorReduction),
                    driveMotorCurrentLimit,
                    1),
            moduleTranslations);

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilograms.of(DriveConstants.robotMassKg))
            .withBumperSize(Inches.of(DriveConstants.bumperLength), Inches.of(DriveConstants.bumperWidth))
            .withTrackLengthTrackWidth(Meters.of(DriveConstants.wheelBase), Meters.of(DriveConstants.trackWidth))
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    DCMotor.getNeoVortex(1),
                    DCMotor.getNeo550(1),
                    DriveConstants.driveMotorReduction,
                    DriveConstants.turnMotorReduction,
                    Volts.of(DriveConstants.driveKs),
                    Volts.of(DriveConstants.turnKs),
                    Meters.of(DriveConstants.wheelRadiusMeters),
                    KilogramSquareMeters.of(.025),
                    DriveConstants.wheelCOF))
            .withGyro(COTS.ofNav2X());

    public static final int ODOMETRY_CACHE_CAPACITY = 5;
    public static final double ODOMETRY_FREQUENCY = 200.0;
    public static final double ODOMETRY_WAIT_TIMEOUT_SECONDS = 0.1;
    public static final int SIMULATION_TICKS_IN_1_PERIOD = 5;
}
