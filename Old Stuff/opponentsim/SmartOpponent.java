package org.ironmaple.simulation.opponentsim;


import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.opponentsim.pathfinding.MapleADStar;
import org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim.ReefscapeOpponentManager;
import org.ironmaple.utils.FieldMirroringUtils;

import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public abstract class SmartOpponent extends SubsystemBase {

    protected Integer id;

    protected DriverStation.Alliance alliance;

    protected SelfControlledSwerveDriveSimulation simulation;

    protected OpponentManager manager;

    protected States currentState;

    protected States previousState = null;

    protected boolean commandInProgress = false;

    protected Pose2d startPose;

    protected Pose2d queeningPose;

    protected StructPublisher<Pose2d> posePublisher;

    protected StringPublisher statePublisher;

    protected Optional<Trigger> isJoystick = Optional.empty();

    protected SendableChooser<Command> behaviorChooser;

    protected Pair<Pose2d, String> targetTask;

    protected PPHolonomicDriveController driveController;

    // MapleSim simulated drive train.
    protected Mass opponentMassKG;
    protected Double opponentMOI;
    protected Distance opponentWheelRadius;
    protected LinearVelocity opponentDriveVelocity;
    protected Double opponentDriveCOF;
    protected DCMotor opponentDriveMotor;
    protected Double opponentDriveCurrentLimit;
    protected Integer opponentNumDriveMotors;
    protected Distance opponentTrackWidth;
    protected Optional<Object> joystick = Optional.empty();

    // PathPlanner configuration
    protected RobotConfig pathplannerConfig;
    protected DriveTrainSimulationConfig driveConfig;
    protected MapleADStar mapleADStar;
    // Static settings
    protected Double joystickDeadzone = 0.1; // TODO Make non-static
    protected Distance driveToPoseCollectTolerance = Inches.of(8);
    protected Distance driveToPoseScoreTolerance = Inches.of(3);
    protected String robotName = "Smart Opponent";

    /**
     * @param id
     * @param alliance
     */
    public void setupOpponent(OpponentManager manager, DriverStation.Alliance alliance, int id) {
        this.id = id;
        this.alliance = alliance;
        this.manager = manager;
        this.startPose = manager.getStartingPose(alliance, id);
        this.queeningPose = manager.getQueeningPose(id);
        this.driveConfig = DriveTrainSimulationConfig.Default();
        this.simulation = new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(
                        driveConfig,
                        queeningPose
                ));
        this.driveController = new PPHolonomicDriveController(new PIDConstants(5.0, 0.0), new PIDConstants(5.0, 0.0));
        this.isJoystick = Optional.of(new Trigger(() -> currentState == States.JOYSTICK));
        this.pathplannerConfig = new RobotConfig(
                    opponentMassKG.in(Kilograms),
                    opponentMOI,
                    new ModuleConfig(
                            opponentWheelRadius.in(Inches),
                            opponentDriveVelocity.in(MetersPerSecond),
                            opponentDriveCOF,
                            opponentDriveMotor,
                            opponentDriveCurrentLimit,
                            opponentNumDriveMotors),
                    opponentTrackWidth.in(Meters));
        SimulatedArena.getInstance().addDriveTrainSimulation(
                this.simulation.getDriveTrainSimulation());
        this.posePublisher =
                NetworkTableInstance.getDefault()
                        .getStructTopic("SmartDashboard/MapleSim/SimulatedRobots/Poses/ "
                                + (alliance.equals(DriverStation.Alliance.Red) ? "Red Alliance " : "Blue Alliance ")
                                + robotName + " " + id + " Pose", Pose2d.struct).publish();
        this.statePublisher =
                NetworkTableInstance.getDefault()
                        .getStringTopic("SmartDashboard/MapleSim/SimulatedRobots/States/ "
                                + (alliance.equals(DriverStation.Alliance.Red) ? "Red Alliance " : "Blue Alliance ")
                                + robotName + " " + id + " Current State").publish();
        this.targetTask = Pair.of(Pose2d.kZero, "Standby");
        this.mapleADStar = new MapleADStar();
        setState(States.STANDBY);
        buildBehaviorChooser(id, alliance);
        manager.registerOpponent(this, alliance);
    }

    /**
     * @param newState
     */
    protected void setState(States newState) {
        currentState = newState;
    }

    /**
     *
     */
    public void runStateCommand(States newState) {
        if (commandInProgress) return;

        Command stateCommand = switch (newState) {
            case STANDBY -> standbyCommand();
            case STARTING -> startingCommand();
            case COLLECT -> collectCommand();
            case SCORE -> scoreCommand();
            case JOYSTICK -> joystickCommand();
            case DEFEND -> defendCommand();
        };

        if (stateCommand != null) {
            commandInProgress = true;
            States thisState = currentState; // States change during the state command.
            stateCommand
                    .finallyDo(() -> {
                        commandInProgress = false;
                        previousState = thisState;
                    }).schedule();
        }
    }

    /**
     * Build the behavior chooser of this opponent robot and send it to the dashboard
     */
    protected void buildBehaviorChooser(int id, DriverStation.Alliance alliance) {
        this.behaviorChooser = new SendableChooser<>();
        // Option to disable the robot
        behaviorChooser.setDefaultOption("Disable", Commands.runOnce(() -> setState(States.STANDBY))
                .andThen(standbyCommand()));

        // Option to auto-cycle random
        behaviorChooser.addOption(
                "Smart Cycle", Commands.runOnce(() -> setState(States.STARTING)));

        // Schedule the command when another behavior is selected
        behaviorChooser.onChange((Command::schedule));

        // Schedule the selected command when teleop starts
        RobotModeTriggers.teleop()
                .onTrue(Commands.runOnce(() -> behaviorChooser.getSelected().schedule()));

        // Disable the robot when the user robot is disabled
        RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> setState(States.STANDBY))
                .andThen(standbyCommand()));

        SmartDashboard.putData("MapleSim/SimulatedRobots/Behaviors/ "
                + (alliance.equals(DriverStation.Alliance.Red) ? "Red Alliance " : "Blue Alliance ")
                + robotName + " " + id + " Behavior", behaviorChooser);
    }

    @Override
    public void simulationPeriodic() {
        if (!commandInProgress && currentState != previousState) {
            runStateCommand(currentState);
            previousState = currentState;
            commandInProgress = true;
            statePublisher.set(currentState.toString());
        }
        posePublisher.set(simulation.getActualPoseInSimulationWorld());
    }

    /**
     *
     */
    protected Command standbyCommand() {
        return Commands.runOnce(() -> simulation.runChassisSpeeds(
                        new ChassisSpeeds(), new Translation2d(), false, false), this)
                .andThen(Commands.runOnce(() -> simulation.setSimulationWorldPose(queeningPose), this))
                .ignoringDisable(true);
    }

    /**
     *
     */
    protected Command startingCommand() {
        return Commands.runOnce(() -> simulation.setSimulationWorldPose(startPose))
                .andThen(Commands.runOnce(() -> simulation.runChassisSpeeds(
                        new ChassisSpeeds(), new Translation2d(), false, false), this))
                .ignoringDisable(true)
                .andThen(Commands.waitSeconds(.25))
                .andThen(() -> setState(States.COLLECT));
    }

    protected Pose2d getNextCollectTarget() {
        targetTask = manager.getNextCollectTarget(alliance, id);
        return targetTask.getFirst();
    }

    /**
     *
     */
    protected Command collectCommand() {

        return (pathfindCommand(ReefscapeOpponentManager.ReefscapeManagerConstants.LEFT_STATION_CENTER_POSE, driveToPoseCollectTolerance).withTimeout(7)
                .alongWith(collect())
                .andThen(Commands.waitSeconds(0.5)))
                .andThen(() -> setState(States.SCORE));
    }

    protected void getNextScoreTarget() {
        targetTask = manager.getNextScoreTarget(alliance, id);
    }

    public Command runChassisSpeeds(ChassisSpeeds speeds) {
        return Commands.run(() -> simulation.runChassisSpeeds(speeds, new Translation2d(), false, false), this);
    }

    /**
     *
     */
    protected Command scoreCommand() {
        getNextScoreTarget();
        return
                (pathfindCommand(targetTask.getFirst(), driveToPoseScoreTolerance).withTimeout(7)
                        .andThen(Commands.waitSeconds(0.5)))
                        .andThen(score())
                        .andThen(() -> setState(States.COLLECT));
    }

    /**
     * @return
     */
    protected Command joystickCommand() {
        if (joystick.isPresent() && joystick.get() instanceof CommandXboxController) {
            return joystickDrive();
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No Joystick found, use .withJoystick() on SmartOpponent instance", false), this);
        }
    }

    /**
     * @return {@link SmartOpponent} for chaining.
     */
    public SmartOpponent withJoystick(CommandXboxController controller) {
        this.joystick = Optional.of(controller);
        behaviorChooser.addOption("Joystick Drive", Commands.runOnce(() -> setState(States.JOYSTICK)));
        return this;
    }

    /**
     *
     */
    protected Command joystickDrive() {
        if (joystick.isPresent()
                && joystick.get() instanceof CommandXboxController controller) {
            return Commands.runOnce(() -> simulation.setSimulationWorldPose(startPose))
                    .andThen(Commands.runOnce(() -> simulation.runChassisSpeeds(
                            new ChassisSpeeds(), new Translation2d(), false, false), this))
                    .ignoringDisable(true)
                    .andThen(Commands.waitSeconds(1))
                    .andThen(Commands.run(() -> {
                        // Calculate field-centric speed from driverstation speed
                        final ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                new ChassisSpeeds(
                                        MathUtil.applyDeadband(-controller.getLeftY(), joystickDeadzone) * simulation.maxLinearVelocity().in(MetersPerSecond),
                                        MathUtil.applyDeadband(-controller.getLeftX(), joystickDeadzone) * simulation.maxLinearVelocity().in(MetersPerSecond),
                                        MathUtil.applyDeadband(-controller.getRightX(), joystickDeadzone) * simulation.maxAngularVelocity().in(RadiansPerSecond)),
                                DriverStation.getAlliance().equals(Optional.of(alliance)) ?
                                        FieldMirroringUtils.getCurrentAllianceDriverStationFacing() :
                                        FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                                                .plus(Rotation2d.k180deg));
                        // Run the field-centric speed
                        simulation.runChassisSpeeds(speeds, new Translation2d(), false, false);
                    }, this));
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No Joystick found, use .withJoystick()[Some overrides may use different methods]", false), this);
        }
    }

    /**
     * This should probably just cycle between 2 poses
     *
     * @return
     */
    protected Command defendCommand() {
        return Commands.runOnce(() -> DriverStation.reportWarning("No defend state implemented", false), this);
    }

    /**
     * This uses PP NavGrid // TODO: Remove Pathplanner dependency, use a more efficient pathfinding algorithm.
     *
     * @param finalPose
     * @return
     */
    protected Command pathfindCommand(Pose2d finalPose, Distance tolerance) {
        DriverStation.reportWarning("Pathfind command has been reached", false);
        mapleADStar.setStartPosition(simulation.getActualPoseInSimulationWorld().getTranslation());
        mapleADStar.setGoalPosition(finalPose.getTranslation());
        mapleADStar.runThread();
        return (Commands.run(() -> {
            // Check if near another opponent.
//            for (SmartOpponent opponent : manager.getOpponents()) {
//                if (opponent != this) {
//                    if (nearPose(opponent.getPose(), tolerance)) {
//                        mapleADStar.setDynamicObstacles(manager.getObstacles(id), simulation.getActualPoseInSimulationWorld().getTranslation());
//                        mapleADStar.runThread();
//                    }
//                    break;
//                }
//            }

            Pose2d currentPose = simulation.getActualPoseInSimulationWorld();
            // Get waypoints
            List<Waypoint> waypoints = mapleADStar.currentWaypoints;
            Translation2d targetTranslation;
            Rotation2d targetRotation;
            // If there's more waypoints, generate a new target.
            if (!waypoints.isEmpty()) {
                DriverStation.reportWarning("Waypoints Count: " + waypoints.size(), false);
                targetTranslation = waypoints.get(0).anchor();
                targetRotation = currentPose.getRotation().interpolate(finalPose.getRotation(), waypoints.size());
                // If the target is close enough to the current pose, remove it.
                if (nearPose(new Pose2d(targetTranslation, targetRotation), tolerance.times(2.0))) {
                    waypoints.remove(0);
                }
            // Else there are no more waypoints, set the target to the final pose.
            } else {
                targetTranslation = finalPose.getTranslation();
                targetRotation = finalPose.getRotation();
            }
            Pose2d targetPose = new Pose2d(targetTranslation, targetRotation);
            PathPlannerTrajectoryState state = new PathPlannerTrajectoryState();
            state.pose = targetPose;
            // Reset controller for best results
            driveController.reset(simulation.getActualPoseInSimulationWorld(), simulation.getActualSpeedsFieldRelative());
            // Calculate field-centric speed to the next waypoint.
            ChassisSpeeds speeds = driveController.calculateRobotRelativeSpeeds(
                    currentPose,
                    state);
            DriverStation.reportWarning("Robot-centric speed: " + speeds.toString(), false);
            // Run the robot-centric speed
            simulation.runChassisSpeeds(speeds, new Translation2d(), false, false);
        }, this)
        .until(() -> mapleADStar.currentWaypoints.isEmpty() && nearPose(finalPose, tolerance))
        .finallyDo(() -> simulation.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false)));
    }

    public Pair<Pose2d, String> getTargetTask() {
        return targetTask;
    }

    public Pose2d getPose() {
        return simulation.getActualPoseInSimulationWorld();
    }

    /**
     * returns false if simulation isn't present
     */
    public boolean nearPose(Pose2d pose, Distance maxDistance) {
            Translation2d robotTranslation = simulation.getActualPoseInSimulationWorld().getTranslation();
            Translation2d goalTranslation = pose.getTranslation();
            double distance = robotTranslation.getDistance(goalTranslation);
            return distance <= maxDistance.in(Meters);
    }

    protected Command collect() {
        return switch (targetTask.getSecond()) {
            case "Station" -> Commands.none();
            case "Floor Intake" -> Commands.none();
            default -> Commands.runOnce(() -> {});
        };
    }

    protected Command score() {
        return switch (targetTask.getSecond()) {
            case "Ball" -> feedShotCommand(null);
            case "Cone" -> Commands.none();
            default -> Commands.runOnce(() -> {});
        };
    }

    protected Command feedShotCommand(GamePieceProjectile projectile) {
        return runOnce(() -> {
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(projectile);
        });
    }

    /**
     * The current state the opponent should be in. Some states may not be used.
     */
    public enum States {
        /**
         * This state puts the {@link SmartOpponent} in it's queening pose.
         */
        STANDBY,
        /**
         * This state puts the robot into the starting pose.
         */
        STARTING,
        /**
         * This state has the opponent decide where then goes to collect a game piece.
         */
        COLLECT,
        /**
         * This state has the opponent decide where then goes and attempts to score a game piece.
         * This typically ignores whether the opponent has properly collected a piece.
         */
        SCORE,
        /**
         * This state lets the opponent be controlled by a controller once setup.
         */
        JOYSTICK,
        /**
         * This state has the opponent defend.
         */
        DEFEND
    }
}
