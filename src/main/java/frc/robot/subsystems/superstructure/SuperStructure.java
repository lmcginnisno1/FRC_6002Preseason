package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GroundPivot.GroundPivot;
import frc.robot.subsystems.GroundPivot.GroundPivotConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristConstants.FlippyPositions;
import frc.robot.subsystems.wrist.WristConstants.SpinnyPositions;
import java.util.*;

public class SuperStructure {
    public enum SuperStructurePose {
        HOME(
                ElevatorConstants.kHome,
                PivotConstants.kHome,
                FlippyPositions.kHome,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        READY(
                ElevatorConstants.kReady,
                PivotConstants.kReady,
                FlippyPositions.kReady,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        INTAKE(
                ElevatorConstants.kIntake,
                PivotConstants.kIntake,
                FlippyPositions.kIntake,
                SpinnyPositions.kIntake,
                GroundPivotConstants.kHome),
        INTAKE_GROUND(
                ElevatorConstants.kIntakeGround,
                PivotConstants.kIntakeGround,
                FlippyPositions.kIntakeGround,
                SpinnyPositions.kIntake,
                GroundPivotConstants.kDeploy),
        TRANSFER(
                ElevatorConstants.kIntakeGround,
                PivotConstants.kIntakeGround,
                FlippyPositions.kIntakeGround,
                SpinnyPositions.kIntake,
                GroundPivotConstants.kTransfer),
        TRANSITION_READY_TO_DEPLOY_L2(
                ElevatorConstants.kHome,
                PivotConstants.kDeployL2,
                FlippyPositions.kHome,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        READY_TO_DEPLOY_L2(
                ElevatorConstants.kDeployL2,
                PivotConstants.kDeployL2,
                FlippyPositions.kDeployL2,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        TRANSITION_DEPLOY_L2_TO_HOME(
                ElevatorConstants.kHome,
                PivotConstants.kDeployL2,
                FlippyPositions.kDeployL2,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        TRANSITION_READY_TO_DEPLOY_L3(
                ElevatorConstants.kHome,
                PivotConstants.kDeployL3,
                FlippyPositions.kHome,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        READY_TO_DEPLOY_L3(
                ElevatorConstants.kDeployL3,
                PivotConstants.kDeployL3,
                FlippyPositions.kDeployL3,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        TRANSITION_DEPLOY_L3_TO_HOME(
                ElevatorConstants.kHome,
                PivotConstants.kDeployL4,
                FlippyPositions.kDeployL4,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        TRANSITION_READY_TO_DEPLOY_L4(
                ElevatorConstants.kHome,
                PivotConstants.kDeployL2,
                FlippyPositions.kHome,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        READY_TO_DEPLOY_L4(
                ElevatorConstants.kDeployL4,
                PivotConstants.kDeployL4,
                FlippyPositions.kDeployL4,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        TRANSITION_DEPLOY_L4_TO_HOME(
                ElevatorConstants.kHome,
                PivotConstants.kDeployL4,
                FlippyPositions.kDeployL4,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        TRANSITION_TO_BARGE(
                ElevatorConstants.kHome,
                PivotConstants.kDeployBarge,
                FlippyPositions.kHome,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        BARGE(
                ElevatorConstants.kDeployBarge,
                PivotConstants.kDeployBarge,
                FlippyPositions.kDeployBarge,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        PROCESSOR(
                ElevatorConstants.kAlgaeProcessor,
                PivotConstants.kAlgaeProcessor,
                FlippyPositions.kAlgaeProcessor,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        ALGAE_GROUND(
                ElevatorConstants.kAlgaeGround,
                PivotConstants.kAlgaeGround,
                FlippyPositions.kAlgaeGround,
                SpinnyPositions.kIntake,
                GroundPivotConstants.kHome),
        TRANSITION_ALGAE_L2(
                ElevatorConstants.kHome,
                PivotConstants.kReadyIntakeAlgae,
                FlippyPositions.kHome,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        ALGAE_L2(
                ElevatorConstants.kReadyIntakeAlgael2,
                PivotConstants.kReadyIntakeAlgae,
                FlippyPositions.kReadyIntakeAlgae,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        TRANSITION_ALGAE_L3(
                ElevatorConstants.kHome,
                PivotConstants.kReadyIntakeAlgael3,
                FlippyPositions.kHome,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        ALGAE_L3(
                ElevatorConstants.kReadyIntakeAlgael3,
                PivotConstants.kReadyIntakeAlgael3,
                FlippyPositions.kReadyIntakeAlgae,
                SpinnyPositions.kHome,
                GroundPivotConstants.kHome),
        ALGAE_OFF_CORAL(
                ElevatorConstants.kAlgaeCoral,
                PivotConstants.kAlgaeCoral,
                FlippyPositions.kAlgaeCoral,
                SpinnyPositions.kIntake,
                GroundPivotConstants.kHome),
        CLIMB(
                ElevatorConstants.kClimb,
                PivotConstants.kClimb,
                FlippyPositions.kClimb,
                SpinnyPositions.kHome,
                GroundPivotConstants.kClimb);

        public final Distance elevatorHeightInches;
        public final Angle pivotAngleRad;
        public final Angle flippyWristAngleRad;
        public final Angle spinnyWristAngleRad;
        public final Angle groundPivotAngleRad;

        SuperStructurePose(
                double elevatorHeightInches,
                double pivotAngleRad,
                double flippyWristAngleRad,
                double spinnyWrstAngleRad,
                double groundPivotAngleRad) {
            this.elevatorHeightInches = Inches.of(elevatorHeightInches);
            this.pivotAngleRad = Radians.of(pivotAngleRad);
            this.spinnyWristAngleRad = Radians.of(spinnyWrstAngleRad);
            this.flippyWristAngleRad = Radians.of(flippyWristAngleRad);
            this.groundPivotAngleRad = Radians.of(groundPivotAngleRad);
        }
    }

    public static List<PoseLink> LINKS = List.of(
            new PoseLink(SuperStructurePose.HOME, SuperStructurePose.READY),
            new PoseLink(SuperStructurePose.HOME, SuperStructurePose.INTAKE_GROUND),
            new PoseLink(SuperStructurePose.HOME, SuperStructurePose.INTAKE),
            new PoseLink(SuperStructurePose.READY, SuperStructurePose.INTAKE_GROUND),
            new PoseLink(SuperStructurePose.READY, SuperStructurePose.INTAKE),
            new PoseLink(SuperStructurePose.INTAKE_GROUND, SuperStructurePose.TRANSFER),
            new PoseLink(SuperStructurePose.TRANSFER, SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L2),
            new PoseLink(SuperStructurePose.TRANSFER, SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L3),
            new PoseLink(SuperStructurePose.TRANSFER, SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L4),
            new PoseLink(SuperStructurePose.INTAKE, SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L2),
            new PoseLink(SuperStructurePose.INTAKE, SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L3),
            new PoseLink(SuperStructurePose.INTAKE, SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L4),
            new PoseLink(SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L2, SuperStructurePose.READY_TO_DEPLOY_L2),
            new PoseLink(SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L3, SuperStructurePose.READY_TO_DEPLOY_L3),
            new PoseLink(SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L4, SuperStructurePose.READY_TO_DEPLOY_L4),
            new PoseLink(SuperStructurePose.READY_TO_DEPLOY_L2, SuperStructurePose.TRANSITION_DEPLOY_L4_TO_HOME),
            new PoseLink(SuperStructurePose.READY_TO_DEPLOY_L3, SuperStructurePose.TRANSITION_DEPLOY_L4_TO_HOME),
            new PoseLink(SuperStructurePose.READY_TO_DEPLOY_L4, SuperStructurePose.TRANSITION_DEPLOY_L4_TO_HOME),
            new PoseLink(SuperStructurePose.READY, SuperStructurePose.ALGAE_GROUND),
            new PoseLink(SuperStructurePose.READY, SuperStructurePose.ALGAE_OFF_CORAL),
            new PoseLink(SuperStructurePose.READY, SuperStructurePose.ALGAE_L2),
            new PoseLink(SuperStructurePose.READY, SuperStructurePose.ALGAE_L3),
            new PoseLink(SuperStructurePose.READY, SuperStructurePose.PROCESSOR),
            new PoseLink(SuperStructurePose.READY, SuperStructurePose.TRANSITION_TO_BARGE),
            new PoseLink(SuperStructurePose.READY_TO_DEPLOY_L2, SuperStructurePose.ALGAE_L2),
            new PoseLink(SuperStructurePose.READY_TO_DEPLOY_L2, SuperStructurePose.ALGAE_L3),
            new PoseLink(SuperStructurePose.READY_TO_DEPLOY_L3, SuperStructurePose.ALGAE_L2),
            new PoseLink(SuperStructurePose.READY_TO_DEPLOY_L3, SuperStructurePose.ALGAE_L3),
            new PoseLink(SuperStructurePose.ALGAE_L2, SuperStructurePose.PROCESSOR),
            new PoseLink(SuperStructurePose.ALGAE_L2, SuperStructurePose.TRANSITION_TO_BARGE),
            new PoseLink(SuperStructurePose.ALGAE_L3, SuperStructurePose.PROCESSOR),
            new PoseLink(SuperStructurePose.ALGAE_L3, SuperStructurePose.TRANSITION_TO_BARGE),
            new PoseLink(SuperStructurePose.TRANSITION_TO_BARGE, SuperStructurePose.BARGE),
            new PoseLink(SuperStructurePose.TRANSITION_TO_BARGE, SuperStructurePose.READY),
            new PoseLink(SuperStructurePose.TRANSITION_TO_BARGE, SuperStructurePose.HOME),
            new PoseLink(SuperStructurePose.ALGAE_L2, SuperStructurePose.READY),
            new PoseLink(SuperStructurePose.ALGAE_L2, SuperStructurePose.HOME),
            new PoseLink(SuperStructurePose.ALGAE_L3, SuperStructurePose.READY),
            new PoseLink(SuperStructurePose.ALGAE_L3, SuperStructurePose.HOME),
            new PoseLink(SuperStructurePose.PROCESSOR, SuperStructurePose.HOME),
            new PoseLink(SuperStructurePose.ALGAE_OFF_CORAL, SuperStructurePose.HOME),
            new PoseLink(SuperStructurePose.ALGAE_GROUND, SuperStructurePose.HOME),
            new PoseLink(SuperStructurePose.READY_TO_DEPLOY_L4, SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L4),
            new PoseLink(SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L4, SuperStructurePose.HOME),
            new PoseLink(SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L4, SuperStructurePose.READY),
            new PoseLink(SuperStructurePose.READY_TO_DEPLOY_L3, SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L3),
            new PoseLink(SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L3, SuperStructurePose.HOME),
            new PoseLink(SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L3, SuperStructurePose.READY),
            new PoseLink(SuperStructurePose.READY_TO_DEPLOY_L2, SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L2),
            new PoseLink(SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L2, SuperStructurePose.HOME),
            new PoseLink(SuperStructurePose.TRANSITION_READY_TO_DEPLOY_L2, SuperStructurePose.READY),
            new PoseLink(SuperStructurePose.TRANSFER, SuperStructurePose.READY),
            new PoseLink(SuperStructurePose.TRANSFER, SuperStructurePose.HOME));

    public record PoseLink(SuperStructurePose pose1, SuperStructurePose pose2) {
        public double timeSeconds() {
            double pivotDiffereneceRad =
                    pose2.pivotAngleRad.minus(pose1.pivotAngleRad).abs(Radians);
            double elevatorDifferenceIN =
                    pose2.elevatorHeightInches.minus(pose1.elevatorHeightInches).abs(Inches);
            double flippyWristDifferenceRad =
                    pose2.flippyWristAngleRad.minus(pose1.flippyWristAngleRad).abs(Radians);
            double spinnyWristDifferenceRad =
                    pose2.spinnyWristAngleRad.minus(pose1.spinnyWristAngleRad).abs(Radians);
            double groundPivotDifferenceRad =
                    pose2.groundPivotAngleRad.minus(pose1.groundPivotAngleRad).abs(Radians);

            double pivotMaxAcc = PivotConstants.kMaxAccel;
            double pivotMaxVel = PivotConstants.kMaxVel;
            double elevatorAcc = ElevatorConstants.kMaxAccel;
            double elevatorVel = ElevatorConstants.kMaxVel;
            double flippyWristMaxAcc = WristConstants.kFlippyMaxAccel;
            double flippyWristMaxVel = WristConstants.kFlippyMaxVel;
            double spinnyWristMaxAcc = WristConstants.kSpinnyMaxAccel;
            double spinnyWristMaxVel = WristConstants.kSpinnyMaxVel;
            double groundPivotMaxAcc = GroundPivotConstants.kMaxAccel;
            double groundPivotMaxVel = GroundPivotConstants.kMaxVel;

            double pivotTime = calculateTimeToSetpoint(pivotDiffereneceRad, pivotMaxAcc, pivotMaxVel);

            double elevatorTime = calculateTimeToSetpoint(elevatorDifferenceIN, elevatorAcc, elevatorVel);

            double flippyWristTime =
                    calculateTimeToSetpoint(flippyWristDifferenceRad, flippyWristMaxAcc, flippyWristMaxVel);
            double spinnyWristTime =
                    calculateTimeToSetpoint(spinnyWristDifferenceRad, spinnyWristMaxAcc, spinnyWristMaxVel);

            double groundPivotTime =
                    calculateTimeToSetpoint(groundPivotDifferenceRad, groundPivotMaxAcc, groundPivotMaxVel);

            double maxTime = 0;
            double[] times = {elevatorTime, pivotTime, flippyWristTime, spinnyWristTime, groundPivotTime};
            for (int i = 0; i < times.length; i++) {
                maxTime = times[i] > maxTime ? times[i] : maxTime;
            }

            return maxTime;
        }

        private static double calculateTimeToSetpoint(double difference, double maxAcc, double maxVel) {
            difference = Math.abs(difference);

            double timeToMaxVel = maxVel / maxAcc;

            double distanceDuringAcc = 0.5 * maxAcc * timeToMaxVel * timeToMaxVel;

            if (difference <= 2 * distanceDuringAcc) return 2 * Math.sqrt(difference / maxAcc);

            double distanceAtConstantVelocity = difference - 2 * distanceDuringAcc;
            double timeAtConstantVel = distanceAtConstantVelocity / maxVel;

            return 2 * timeToMaxVel + timeAtConstantVel;
        }

        public Optional<SuperStructurePose> otherEdge(SuperStructurePose oneEdge) {
            if (oneEdge == pose1) return Optional.of(pose2);
            if (oneEdge == pose2) return Optional.of(pose1);
            else return Optional.empty();
        }
    }

    private final Pivot pivot;
    private final Elevator elevator;
    private final Wrist wrist;
    private final GroundPivot groundPivot;

    private SuperStructurePose currentPose;
    private SuperStructurePose goal;
    public final Trigger atReference;

    public SuperStructure(Pivot pivot, Elevator elevator, Wrist wrist, GroundPivot groundPivot) {
        this.pivot = pivot;
        this.elevator = elevator;
        this.wrist = wrist;
        this.groundPivot = groundPivot;
        this.goal = this.currentPose = SuperStructurePose.HOME;

        atReference = new Trigger(elevator::getInPosition)
                .and(pivot::getInPosition)
                .and(wrist::getFlippyInPosition)
                .and(wrist::getSpinnyInPosition)
                .and(groundPivot::inPosition);
        atReference.onTrue(Commands.runOnce(() -> currentPose = goal));

        new Trigger(DriverStation::isTeleop).onTrue(moveToPose(SuperStructurePose.HOME));

        wwristUpCommand().schedule();
    }

    private Command runPose(SuperStructurePose pose) {
        return elevator.setGoal(pose.elevatorHeightInches)
                .alongWith(pivot.setGoal(pose.pivotAngleRad))
                .alongWith(wrist.setFlippyGoal(pose.flippyWristAngleRad))
                .alongWith(wrist.setSpinnyGoal(pose.spinnyWristAngleRad))
                .alongWith(groundPivot.setGoal(pose.groundPivotAngleRad))
                .beforeStarting(Commands.print("Super Structure/Moving to pose: " + pose.name()))
                .finallyDo(interrupted -> {
                    currentPose = pose;
                    if (interrupted)
                        System.out.println("Super Structure/Interrupted while running to pose: " + pose.name());
                    else System.out.println("Super Structure/Reached pose: " + pose.name());
                });
    }

    public Command moveToPose(SuperStructurePose pose) {
        return Commands.defer(() -> generateMoveToPoseCommand(pose), Set.of(pivot, elevator, wrist))
                .beforeStarting(() -> goal = pose);
    }

    private Command generateMoveToPoseCommand(SuperStructurePose pose) {
        Optional<List<SuperStructurePose>> trajectory = getTrajectory(pose);
        if (trajectory.isEmpty()) return this.runPose(pose);
        return Commands.sequence(trajectory.get().stream().map(this::runPose).toArray(Command[]::new));
    }

    public SuperStructurePose currentPose() {
        return currentPose;
    }

    public SuperStructurePose targetPose() {
        return goal;
    }

    private static final int loopNumLimit = 100;

    public static Optional<List<SuperStructurePose>> getTrajectory(
            SuperStructurePose startingPose, SuperStructurePose targetPose) {
        if (startingPose.equals(targetPose)) return Optional.of(new ArrayList<>());

        Set<SuperStructurePose> unvisited = new HashSet<>(Set.of(SuperStructurePose.values()));

        Map<SuperStructurePose, PoseLink> minimumTimePathToNode = new HashMap<>();
        Map<SuperStructurePose, Double> minimumTimeToPose = new HashMap<>();
        PriorityQueue<PoseLink> linksToExamine = new PriorityQueue<>(Comparator.comparingDouble(PoseLink::timeSeconds));
        for (SuperStructurePose pose : SuperStructurePose.values())
            minimumTimeToPose.put(pose, Double.POSITIVE_INFINITY);
        minimumTimeToPose.put(startingPose, 0.0);

        SuperStructurePose currentNode = startingPose;

        int i;
        for (i = 0; i < loopNumLimit; i++) {
            for (PoseLink link : LINKS) {
                Optional<SuperStructurePose> otherNode = link.otherEdge(currentNode);
                if (otherNode.isEmpty()) continue;
                linksToExamine.add(link);

                double newTime = minimumTimeToPose.get(currentNode) + link.timeSeconds();
                if (newTime < minimumTimeToPose.get(otherNode.get())) {
                    minimumTimePathToNode.put(otherNode.get(), link);
                    minimumTimeToPose.put(otherNode.get(), newTime);
                }
            }

            unvisited.remove(currentNode);

            PoseLink linkToExamine;
            while ((linkToExamine = linksToExamine.poll()) != null) {
                if (unvisited.contains(linkToExamine.pose1)) {
                    currentNode = linkToExamine.pose1;
                    break;
                }
                if (unvisited.contains(linkToExamine.pose2)) {
                    currentNode = linkToExamine.pose2;
                    break;
                }
            }
            if (linkToExamine == null) break;
        }

        List<SuperStructurePose> trajectory = new ArrayList<>();
        SuperStructurePose tmp = targetPose;
        System.out.println("<-- tracing trajectory: -->");
        for (int j = 0; j < loopNumLimit; j++) {
            System.out.println("    tracing node: " + tmp);
            trajectory.add(0, tmp);
            if (tmp == startingPose) {
                System.out.println("Successfully planned trajectory: " + printTrajectory(trajectory));
                return Optional.of(trajectory);
            }
            if (!minimumTimePathToNode.containsKey(tmp)) {
                DriverStation.reportError("Internal Error while tracing back trajectory", true);
                return Optional.empty();
            }
            Optional<SuperStructurePose> otherEdge =
                    minimumTimePathToNode.get(tmp).otherEdge(tmp);
            if (otherEdge.isEmpty()) {
                DriverStation.reportError("Internal Error while tracing back trajectory", true);
                return Optional.empty();
            }
            tmp = otherEdge.get();
        }

        DriverStation.reportError(
                "Internal Error: destination reached, but cannot traceback trajectory in " + loopNumLimit
                        + " iterations",
                true);
        return Optional.empty();
    }

    private static String printTrajectory(List<SuperStructurePose> trajectory) {
        if (trajectory.isEmpty()) return "(Empty Trajectory)";
        StringBuilder message = new StringBuilder();
        for (int i = 0; i < trajectory.size() - 1; i++)
            message.append(trajectory.get(i).name()).append(" -> ");
        message.append(trajectory.get(trajectory.size() - 1).name());
        return message.toString();
    }

    public Optional<List<SuperStructurePose>> getTrajectory(SuperStructurePose targetPose) {
        return getTrajectory(currentPose, targetPose);
    }

    private void testTrajectoryGen() {
        long t0 = System.currentTimeMillis();
        for (int i = 0; i < 10; i++) getTrajectory(SuperStructurePose.HOME, SuperStructurePose.HOME);
        System.out.println("tried 10 plans, took " + (System.currentTimeMillis() - t0) + " ms");
    }

    public Command wwristUpCommand() {
        return Commands.run(this::testTrajectoryGen)
                .until(DriverStation::isEnabled)
                .withTimeout(10)
                .ignoringDisable(true);
    }
}
