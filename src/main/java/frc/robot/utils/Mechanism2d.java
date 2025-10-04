package frc.robot.utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Mechanism2d {
    private final LoggedMechanism2d elevatorMechanism;
    private final LoggedMechanismRoot2d elevatorRoot;
    private final LoggedMechanismLigament2d elevatorShaft;
    private final LoggedMechanismLigament2d wrist;

    private final LoggedMechanism2d groundPivotMechanism;
    private final LoggedMechanismRoot2d groundPivotRoot;
    private final LoggedMechanismLigament2d groundPivotArm;

    public Mechanism2d() {
        elevatorMechanism = new LoggedMechanism2d(Units.inchesToMeters(6), Units.inchesToMeters(60));
        elevatorRoot = elevatorMechanism.getRoot("elevatorRoot", Units.inchesToMeters(-5), Units.inchesToMeters(11));
        elevatorShaft = new LoggedMechanismLigament2d("elevator", Units.inchesToMeters(24), 90);
        wrist = new LoggedMechanismLigament2d("wrist", Units.inchesToMeters(10), 0);
        elevatorRoot.append(elevatorShaft);
        elevatorShaft.append(wrist);

        groundPivotMechanism = new LoggedMechanism2d(Units.inchesToMeters(15.5), Units.inchesToMeters(8.5));
        groundPivotRoot =
                groundPivotMechanism.getRoot("groundPivotRoot", Units.inchesToMeters(11), Units.inchesToMeters(9));
        groundPivotArm = new LoggedMechanismLigament2d(
                "groundPivotArm", Units.inchesToMeters(13.5), -90, 10, new Color8Bit(235, 137, 52));
        groundPivotRoot.append(groundPivotArm);
    }

    public void setPivotAngle(double deg) {
        elevatorShaft.setAngle(deg);
    }

    public void setElevatorLength(double inches) {
        elevatorShaft.setLength(inches);
    }

    public void setWristAngle(double deg) {
        wrist.setAngle(deg);
    }

    public void setGroundPivotAngle(double deg) {
        groundPivotArm.setAngle(deg);
    }

    public void update() {
        Logger.recordOutput("OutakeAssembly", elevatorMechanism);
        Logger.recordOutput("GroundPivotMechanism", groundPivotMechanism);
    }
}
