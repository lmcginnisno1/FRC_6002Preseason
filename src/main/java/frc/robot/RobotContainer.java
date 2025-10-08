package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.*;
import frc.robot.subsystems.GroundIntake.*;
import frc.robot.subsystems.GroundPivot.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.pivot.*;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.wrist.*;
import frc.robot.utils.Mechanism2d;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    public static final boolean SIMULATE_AUTO_PLACEMENT_INACCURACY = true;

    public final LoggedPowerDistribution powerDistributionLog;
    // Subsystems
    public final Drive drive;
    public final Pivot pivot;
    public final Elevator elevator;
    public final Wrist wrist;
    public final GroundPivot groundPivot;
    public final GroundIntake groundIntake;
    // public final Vision vision;
    public final Mechanism2d mechanism2d = new Mechanism2d();
    // Controller
    // public final DriverMap driver = new DriverMap.LeftHandedXbox(0);
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);

    // Simulated drive
    private final SwerveDriveSimulation driveSimulation;
    public final SuperStructure superStructure;

    private final Field2d field = new Field2d();
    private LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("autoChooser");

    // The container for the robot. Contains subsystems, IO devices, and commands.
    public RobotContainer() {

        switch (Robot.CURRENT_ROBOT_MODE) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                driveSimulation = null;

                drive = new Drive(
                        new GyroIONavX(),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(4),
                        (pose) -> {});

                pivot = new Pivot(new PivotIOSpark(), null);
                elevator = new Elevator(new ElevatorIOSpark(), null);
                wrist = new Wrist(new WristIOSpark(), null);
                groundPivot = new GroundPivot(new GroundPivotIOSpark(), null);
                groundIntake = new GroundIntake(new GroundIntakeIOSpark());

                powerDistributionLog = LoggedPowerDistribution.getInstance(1, PowerDistribution.ModuleType.kRev);
            }

            case SIM -> {
                // create a maple-sim swerve drive simualtion instance
                this.driveSimulation =
                        new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                // add the simulated drivetrain to the simulation field
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                // Sim robot, instantiate physiscs sim to IO implementations

                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);

                pivot = new Pivot(new PivotIOSim(), mechanism2d);
                elevator = new Elevator(new ElevatorIOSim(), mechanism2d);
                wrist = new Wrist(new WristIOSim(), mechanism2d);
                groundPivot = new GroundPivot(new GroundPivotIOSim(), mechanism2d);
                groundIntake = new GroundIntake(new GroundIntakeIOSim());

                powerDistributionLog = LoggedPowerDistribution.getInstance();

                SimulatedArena.getInstance().resetFieldForAuto();
            }

            default -> {
                this.driveSimulation = null;

                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});

                pivot = new Pivot(new PivotIO() {}, null);
                elevator = new Elevator(new ElevatorIO() {}, null);
                wrist = new Wrist(new WristIO() {}, null);
                groundPivot = new GroundPivot(new GroundPivotIO() {}, null);
                groundIntake = new GroundIntake(new GroundIntakeIO() {});

                powerDistributionLog = LoggedPowerDistribution.getInstance();
            }
        }

        this.superStructure = new SuperStructure(pivot, elevator, wrist, groundPivot);

        configureButtonBindings();

        SmartDashboard.putData("Field", field);

        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    }

    private Command autonomousCommand = autoChooser.get();

    public void configureButtonBindings() {
        // driver.a().onTrue(new InstantCommand (()-> groundPivot.setGoal(GroundPivotConstants.kDeploy)));
        // driver.b().onTrue(new InstantCommand(()-> groundPivot.setGoal(GroundPivotConstants.kHome)));

        // driver.x().onTrue(new InstantCommand(()-> wrist.setFlippyGoal(WristConstants.FlippyPositions.kHome)));
        // driver.y().onTrue(new InstantCommand(()-> wrist.setFlippyGoal(WristConstants.FlippyPositions.kDeployL4)));

        driver.povUp().onTrue(wrist.getFlippySysId().quasistatic(Direction.kForward));
        driver.povLeft().onTrue(wrist.getFlippySysId().quasistatic(Direction.kReverse));
        driver.povDown().onTrue(wrist.getFlippySysId().dynamic(Direction.kForward));
        driver.povRight().onTrue(wrist.getFlippySysId().dynamic(Direction.kReverse));

        driver.povUp().onTrue(groundPivot.getSysId().quasistatic(Direction.kForward));
        driver.povLeft().onTrue(groundPivot.getSysId().quasistatic(Direction.kReverse));
        driver.povDown().onTrue(groundPivot.getSysId().dynamic(Direction.kForward));
        driver.povRight().onTrue(groundPivot.getSysId().dynamic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autonomousCommand;
    }

    public void updateFieldSimAndDisplay() {
        if (driveSimulation == null) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));

        mechanism2d.update();
    }
}
