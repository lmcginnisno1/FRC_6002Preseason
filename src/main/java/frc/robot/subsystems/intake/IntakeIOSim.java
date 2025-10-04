package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim {
    private final IntakeSimulation intakeSimulation;

    public IntakeIOSim(AbstractDriveTrainSimulation drivetrain) {
        // Here, create the intake simulation with respect to the intake on your real robot
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                // Specify the type of game pieces that the intake can collect
                "Algae",
                // Specify the drivetrain to which this intake is attached
                drivetrain,
                // Specify width of the intake
                Meters.of(.4),
                // specify length of the intake
                Meters.of(.1),
                // The intake is mounted on the back side of the chassis
                IntakeSimulation.IntakeSide.FRONT,
                // The intake can hold up to 1 algae
                1);
    }

    public void setRunning(boolean runIntake) {
        if (runIntake)
            intakeSimulation
                    .startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with
        // game pieces
        else
            intakeSimulation
                    .stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
    }

    public boolean haveAlgae() {
        return intakeSimulation.getGamePiecesAmount() != 0; // True if there is a game piece in the intake
    }

    public void removeAlgae() {
        intakeSimulation.obtainGamePieceFromIntake();
    }
}
