package org.ironmaple.simulation.opponentsim;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.opponentsim.configs.SmartOpponentConfig;
import org.ironmaple.simulation.opponentsim.pathfinding.MapleADStar;
import org.ironmaple.utils.FieldMirroringUtils;

import java.util.*;

import static edu.wpi.first.units.Units.*;

public abstract class SmartOpponent extends SubsystemBase {
    /// Publishers
    protected StringPublisher statePublisher;
    protected StructPublisher<Pose2d> posePublisher;
    // String used in telemetry for alliance.
    protected String allianceString;
    /// The SmartOpponentConfig to use.
    protected SmartOpponentConfig config;
    /// The manipulator simulation.
    protected ManipulatorSim manipulatorSim;
    /// The drivetrain simulation.
    protected SelfControlledSwerveDriveSimulation drivetrainSim;
    /// The pathplanner config
    protected RobotConfig pathplannerConfig;
    /// Pathplanner HolonomicDriveController
    protected HolonomicDriveController driveController;
    /// Opponent Management
    // Opponents current and last states.
    public String currentState;
    public String lastState;
    // Pathfinding class cloned for modification.
    private final MapleADStar mapleADStar;
    // Behavior Chooser Publisher
    private final StringPublisher selectedBehaviorPublisher;

    public SmartOpponent(SmartOpponentConfig config) {
        /// Create and verify config.
        this.config = config;
        config.validConfig(); // Throw an error if the config is invalid.
        /// Initialize simulations
        this.drivetrainSim = config.chassis.updateDriveTrainSim(config.initialPose);
        this.pathplannerConfig = config.chassis.updatePathplannerConfig();
        this.driveController = new HolonomicDriveController(
                new PIDController(5, 0, 0),
                new PIDController(5, 0, 0),
                new ProfiledPIDController(5, 0, 0,
                        new TrapezoidProfile.Constraints(
                                config.chassis.maxLinearVelocity.in(MetersPerSecond),
                                config.chassis.maxAngularVelocity.in(DegreesPerSecond))));
        // Cloned Pathfinder for use here.
        this.mapleADStar = new MapleADStar();
        // Alliance string for telemetry.
        this.allianceString = DriverStation.Alliance.Blue.equals(config.alliance) ? "Blue Alliance/" : "Red Alliance/";
        // NetworkTable setup.
        this.statePublisher = NetworkTableInstance.getDefault()
                .getStringTopic(config.telemetryPath + "SimulatedOpponents/States/" + allianceString
                        + config.name + "'s Current State").publish();
        this.posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(config.telemetryPath + "SimulatedOpponents/Poses/" + allianceString
                        + config.name + "'s Pose2d", Pose2d.struct).publish();
        this.selectedBehaviorPublisher = NetworkTableInstance.getDefault()
                .getTable(config.smartDashboardPath + "SimulatedOpponents/Behaviors/" + allianceString + config.name + "'s Behaviors")
                .getStringTopic("selected")
                .publish();
        /// Adds the required states to run the {@link org.ironmaple.simulation.opponentsim.SmartOpponent}.
        config.getStates().put("Standby", standbyState());
        config.getStates().put("Starting", startingState("Collect"));
        config.getStates().put("Collect", collectState());
        config.getStates().put("Score", scoreState());
        setState("Standby");
        /// Adds options to the behavior sendable chooser.
        config.addBehavior("Disabled", runState("Standby", true), true);
        config.addBehavior("Enabled", runState("Starting", true), false);
        // Update the chooser and then publish it.
        SmartDashboard.putData(config.smartDashboardPath + "SimulatedOpponents/Behaviors/" + allianceString
                + config.name + "'s Behaviors", config.updateBehaviorChooser());
        /// Run behavior command when changed.
        config.getBehaviorChooser().onChange(Command::schedule);
        /// Finally, add our simulation
        SimulatedArena.getInstance().addDriveTrainSimulation(drivetrainSim.getDriveTrainSimulation());
        if (true) {
            RobotModeTriggers.teleop().onTrue(setSelectedBehavior("Enabled"));
            RobotModeTriggers.disabled().onTrue(setSelectedBehavior("Disabled"));
        }
    }

    public Command setSelectedBehavior(String behavior) {
        return runOnce(() -> selectedBehaviorPublisher.set(behavior));
    }

    /**
     * The standby state to run.
     *
     * @return a Command that runs the state.
     */
    protected Command standbyState() {
        return runOnce(() -> drivetrainSim.runChassisSpeeds(
                new ChassisSpeeds(), new Translation2d(), false, false))
                .andThen(runOnce(() -> drivetrainSim.setSimulationWorldPose(config.initialPose)))
                .ignoringDisable(true);
    }

    /**
     * The starting state to run.
     *
     * @return a Command that runs the state.
     */
    protected Command startingState(String nextState) {
        return runOnce(() -> drivetrainSim.runChassisSpeeds(
                new ChassisSpeeds(), new Translation2d(), false, false))
                .andThen(runOnce(() -> drivetrainSim.setSimulationWorldPose(config.initialPose)))
                .andThen(Commands.waitSeconds(0.25))
                .andThen(() -> setState(nextState))
                .ignoringDisable(true);
    }

    /**
     * The collect state to run.
     *
     * @return a runnable that runs the state.
     */
    abstract protected Command collectState();

    /**
     * The score state to run.
     *
     * @return a runnable that runs the state.
     */
    abstract protected Command scoreState();

    @Override
    public void simulationPeriodic() {
        boolean commandInProgress = getCurrentCommand() != null && getCurrentCommand().isFinished();
        if (!commandInProgress && !Objects.equals(lastState, currentState)) {
            runState(currentState, true);
            lastState = currentState;
        }
        drivetrainSim.periodic();
        statePublisher.set(currentState);
        posePublisher.set(drivetrainSim.getActualPoseInSimulationWorld());
    }

    /**
     * Sets the current state of the robot.
     * This waits it's turn patiently for the command to finish.
     *
     * @param state The state to set.
     * @return this, for chaining.
     */
    public SmartOpponent setState(String state) {
        currentState = state;
        return this;
    }

    /**
     * Runs a state as a command.
     *
     * @param state      The state to run.
     * @param forceState Whether to force the state to run even if it is already running.
     * @return The command to run the state.
     */
    public Command runState(String state, boolean forceState) {
        boolean commandInProgress = getCurrentCommand() != null && getCurrentCommand().isFinished();
        if (!forceState) {
            // If already in the state or a command is in progress, return nothing.
            if (currentState.equals(state) || commandInProgress) {
                setState(state); // Make state wait for command to finish.
                return Commands.none();
            }
        } else {
            if (getCurrentCommand() != null) {
                getCurrentCommand().cancel();
            }
        }
        Command desiredState = config.getStates().get(state);
        return Objects.requireNonNullElseGet(desiredState, Commands::none);
    }

    /**
     * Pathfinds to a target pose.
     *
     * @param targetPose The target pose.
     * @return A command to pathfind to the target pose.
     */
    protected Command pathfind(Pose2d targetPose) {
        /// Set up the pathfinder
        mapleADStar.setStartPosition(drivetrainSim.getActualPoseInSimulationWorld().getTranslation());
        mapleADStar.setGoalPosition(targetPose.getTranslation());
        mapleADStar.runThread();
        return run(() -> {
            Pose2d currentPose = drivetrainSim.getActualPoseInSimulationWorld();
            List<Waypoint> waypoints = mapleADStar.currentWaypoints;
            Translation2d targetTranslation;
            Rotation2d targetRotation;
            /// If waypoints exist make the next one our target.
            if (!waypoints.isEmpty()) {
                targetTranslation = waypoints.get(0).anchor();
                targetRotation = currentPose.getRotation().interpolate(targetPose.getRotation(), waypoints.size());
                /// If we are close enough, remove it and move to the next waypoint.
                if (currentPose.getTranslation().getDistance(targetTranslation) < config.chassis.driveToPoseTolerance.in(Meters)) {
                    waypoints.remove(0);
                }
                /// Else, we set our target to our desired final pose.
            } else {
                targetTranslation = targetPose.getTranslation();
                targetRotation = targetPose.getRotation();
            }
            /// Create a {@link Pose2d} from our waypoint targets.
            Pose2d waypointTarget = new Pose2d(targetTranslation, targetRotation);
            /// Calculate our chassis speeds.
            ChassisSpeeds speeds = driveController.calculate(
                    currentPose,
                    waypointTarget,
                    config.chassis.maxLinearVelocity.in(MetersPerSecond),
                    targetRotation);
            drivetrainSim.runChassisSpeeds(speeds, new Translation2d(), false, false);
        })
                /// Once we have no more targets, finish the command.
                .until(() -> {
                    List<Waypoint> waypoints = mapleADStar.currentWaypoints;
                    return waypoints.isEmpty() && nearPose(targetPose, config.chassis.driveToPoseTolerance.in(Meters));
                })
                .finallyDo(() -> drivetrainSim.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false));
    }

    /**
     * Gets a random pose from a map.
     *
     * @param poseMap The map to get a pose from.
     * @return A random pose from the map.
     */
    public Pose2d getRandomFromMap(Map<String, Pose2d> poseMap) {
        return poseMap.values().toArray(new Pose2d[0])[new Random().nextInt(poseMap.size())];
    }

    public boolean nearPose(Pose2d pose, double tolerance) {
        Translation2d thisTranslation = drivetrainSim.getActualPoseInSimulationWorld().getTranslation();
        Translation2d otherTranslation = pose.getTranslation();
        return thisTranslation.getDistance(otherTranslation) < tolerance;
    }

    /**
     * If robot is red, returns pose as is. If the robot is blue, it returns the flipped pose.
     *
     * @param pose The pose to flip.
     * @return The flipped pose.
     */
    public Pose2d ifShouldFlip(Pose2d pose) {
        if (config.alliance == DriverStation.Alliance.Red) {
            return pose;
        } else {
            return new Pose2d(
                    FieldMirroringUtils.flip(pose.getTranslation()),
                    FieldMirroringUtils.flip(pose.getRotation()));
        }
    }

    protected static class ManipulatorSim extends SubsystemBase {
        private final Map<String, org.ironmaple.simulation.IntakeSimulation> intakeSimulations;
        private final Map<String, GamePieceProjectile> projectileSimulations;

        /**
         * Creates a new manipulator simulation.
         */
        public ManipulatorSim()
        {
            this.intakeSimulations = new HashMap<>();
            this.projectileSimulations = new HashMap<>();
        }

        /**
         * Adds an intake simulation to the manipulator simulation.
         *
         * @param name The name of the simulation.
         * @param intakeSimulation The simulation to add.
         * @return this, for chaining.
         */
        public ManipulatorSim addIntakeSimulation(String name, IntakeSimulation intakeSimulation)
        {
            this.intakeSimulations.put(name, intakeSimulation);
            return this;
        }

        /**
         * Adds a projectile simulation to the manipulator simulation.
         *
         * @param name The name of the simulation.
         * @param projectileSimulation The simulation to add.
         * @return this, for chaining.
         */
        public ManipulatorSim addProjectileSimulation(String name, GamePieceProjectile projectileSimulation)
        {
            this.projectileSimulations.put(name, projectileSimulation);
            return this;
        }

        /**
         * Gets an intake simulation from the manipulator simulation.
         *
         * @param name The name of the simulation.
         * @return The simulation.
         */
        public IntakeSimulation getIntakeSimulation(String name)
        {
            return this.intakeSimulations.get(name);
        }

        /**
         * Gets a projectile simulation from the manipulator simulation.
         *
         * @param projectileName The name of the simulation.
         * @return The simulation.
         */
        public GamePieceProjectile getProjectileSimulation(String projectileName)
        {
            return this.projectileSimulations.get(projectileName);
        }

        /**
         * Adds a projectile to the simulation.
         *
         * @param projectileName The name of the projectile simulation.
         * @return a command to add the game piece projectile to the simulation.
         */
        public Command feedShot(String projectileName)
        {
            return Commands.runOnce(() -> {
                SimulatedArena.getInstance().addGamePieceProjectile(getProjectileSimulation(projectileName));
            });
        }
    }
}
