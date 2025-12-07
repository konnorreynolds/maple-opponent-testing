package org.ironmaple.simulation.opponentsim;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public abstract class SmartOpponent extends SubsystemBase
{
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
    protected PPHolonomicDriveController driveController;
    // Pathfinding class cloned for modification.
    private final MapleADStar mapleADStar;
    // Behavior Chooser Publisher
    private StringPublisher selectedBehaviorPublisher;

    public SmartOpponent(SmartOpponentConfig config)
    {
        /// Create and verify config.
        this.config = config;
        config.validConfig(); // Throw an error if the config is invalid.
        this.driveController = new PPHolonomicDriveController(
                new PIDConstants(5),
                new PIDConstants(5));
        // Cloned Pathfinder for use here.
        this.mapleADStar = new MapleADStar();
        finalizeOpponent();
    }

    /**
     * Used to update telemetry and more at the end to make sure there are no additional changes.
     * Call again if you want to make a change to something major live.
     * It's recommended to check first.
     */
    protected void finalizeOpponent()
    {
        /// Initialize simulations
        this.drivetrainSim = config.chassis.createDriveTrainSim(config.queeningPose);
        this.pathplannerConfig = config.chassis.updatePathplannerConfig();
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
        config.addState("Standby", standbyState());
        config.addState("Starting", startingState("Collect"));
        config.addState("Collect", collectState());
        config.addState("Score", scoreState());
        setState("Standby");
        /// Adds options to the behavior sendable chooser.
        config.addBehavior("Disabled", runState("Standby", true), true);
        config.addBehavior("Enabled", runState("Starting", true));
        // Update the chooser and then publish it.
        SmartDashboard.putData(config.smartDashboardPath + "SimulatedOpponents/Behaviors/" + allianceString
                + config.name + "'s Behaviors", config.updateBehaviorChooser());
        /// Run behavior command when changed.
        config.getBehaviorChooser().onChange(Command::schedule);
        /// Finally, add our simulation
        SimulatedArena.getInstance().addDriveTrainSimulation(drivetrainSim.getDriveTrainSimulation());
        if (config.isAutoEnable) {
            RobotModeTriggers.teleop().onTrue(runOnce(() -> config.getBehaviorChooser().getSelected().schedule()));
            RobotModeTriggers.disabled().onTrue(runState("Standby", true));
        }
    }

    protected Command setSelectedBehavior(String behavior)
    {
        return runOnce(() -> selectedBehaviorPublisher.set(behavior));
    }

    /**
     * The standby state to run.
     *
     * @return a Command that runs the state.
     */
    protected Command standbyState()
    {
        return runOnce(() -> drivetrainSim.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false))
                .andThen(runOnce(() -> drivetrainSim.setSimulationWorldPose(config.queeningPose)))
                .ignoringDisable(true);
    }

    /**
     * The starting state to run.
     *
     * @return a Command that runs the state.
     */
    protected Command startingState(String nextState)
    {
        return runOnce(() -> drivetrainSim.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false))
                .andThen(runOnce(() -> drivetrainSim.setSimulationWorldPose(config.initialPose)))
                .andThen(Commands.waitSeconds(0.25))
                .finallyDo(() -> setState(nextState))
                .ignoringDisable(config.isAutoEnable); // Only move to the field while disabled if autoEnable is off.
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

    Debouncer db = new Debouncer(1);
    @Override
    public void simulationPeriodic()
    {
        boolean commandInProgress = getCurrentCommand() != null && !getCurrentCommand().isFinished();
        DriverStation.reportError("Command in Progress: " + commandInProgress, false);
        DriverStation.reportWarning("Current State: " + config.currentState, false);
        DriverStation.reportWarning("Desired State: " + config.desiredState, false);
        if (!commandInProgress && !Objects.equals(config.currentState, config.desiredState)) {
            runState(config.desiredState, false).schedule();
        }
        drivetrainSim.periodic();
        statePublisher.set(config.currentState);
        posePublisher.set(drivetrainSim.getActualPoseInSimulationWorld());
    }

    /**
     * Sets the current state of the robot.
     * This waits it's turn patiently for the command to finish.
     *
     * @param state The state to set.
     * @return this, for chaining.
     */
    protected SmartOpponent setState(String state) {
         config.desiredState = state;
        return this;
    }

    /**
     * Runs a state as a command.
     *
     * @param state      The state to run.
     * @param forceState Whether to force the state to run even if it is already running.
     * @return The command to run the state.
     */
    protected Command runState(String state, boolean forceState) {
        /// If forceState cancel any commands.
        if (forceState) {
            if (getCurrentCommand() != null) {
                getCurrentCommand().cancel();
            }
        } else
        { /// Don't force the state. If there's a command running, wait.
            // If already in the state or a command is in progress, return nothing.
            if (config.currentState.equals(state)
                    || getCurrentCommand() != null
                    || (getCurrentCommand() != null  && !getCurrentCommand().isFinished())
                    && (!RobotModeTriggers.disabled().getAsBoolean() && config.isAutoEnable)) {
                setState(state); // Make state wait for command to finish.
                return Commands.none();
            }
        }
        /// If nothing is in the way, schedule our state.
        return config.getStates().get(state);
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
            /// Create a desired state
            PathPlannerTrajectoryState state = new PathPlannerTrajectoryState();
            state.pose = waypointTarget;
            /// Calculate our chassis speeds.
            ChassisSpeeds speeds = driveController.calculateRobotRelativeSpeeds(
                    currentPose,
                    state);
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
     * A {@link Command} to drive the {@link SmartOpponent}.
     * Meant to be used for joystick drives.
     *
     * @param chassisSpeeds speeds supplier to run the robot.
     * @return {@link Command} to drive the {@link SmartOpponent}.
     */
    protected Command drive(Supplier<ChassisSpeeds> chassisSpeeds, boolean fieldCentric) {
        final Supplier<ChassisSpeeds> updatedSpeed = () ->
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        chassisSpeeds.get(),
                        config.alliance.equals(DriverStation.Alliance.Blue)
                        ? Rotation2d.kZero
                                : Rotation2d.k180deg);
        return run(() -> {
            DriverStation.reportError("DRIVE FORWARDING", false);
            drivetrainSim.runChassisSpeeds(updatedSpeed.get(), new Translation2d(), fieldCentric, false);});
    }

    /**
     * Gets a random pose from a map.
     *
     * @param poseMap The map to get a pose from.
     * @return A random pose from the map.
     */
    protected Pose2d getRandomFromMap(Map<String, Pose2d> poseMap) {
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
    protected Pose2d ifShouldFlip(Pose2d pose) {
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