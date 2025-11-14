package org.ironmaple.simulation.opponentsim;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
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
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.utils.FieldMirroringUtils;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;

import static edu.wpi.first.units.Units.*;

public abstract class SmartOpponent extends SubsystemBase {

    protected Optional<Integer> id = Optional.empty();

    protected Optional<DriverStation.Alliance> alliance = Optional.empty();

    protected Optional<OpponentManager> manager = Optional.empty();

    protected Optional<SelfControlledSwerveDriveSimulation> simulation = Optional.empty();

    protected Optional<States> currentState = Optional.empty();

    protected Optional<States> previousState = Optional.empty();

    protected boolean commandInProgress = false;

    protected Optional<Pose2d> startPose = Optional.empty();

    protected Optional<Pose2d> queeningPose = Optional.empty();

    protected Optional<StructPublisher<Pose2d>> posePublisher = Optional.empty();

    protected Optional<StringPublisher> statePublisher = Optional.empty();

    protected Optional<Trigger> isJoystick = Optional.empty();

    protected Optional<SendableChooser<Command>> behaviorChooser = Optional.empty();

    protected Optional<Pose2d> target = Optional.empty();

    protected Optional<List<OpponentManager.ObstacleRect>> obstacles = Optional.empty();

    // MapleSim simulated drive train.
    protected Optional<DriveTrainSimulationConfig> driveConfig = Optional.empty();
    protected Optional<Mass> opponentMassKG = Optional.empty();
    protected Optional<Double> opponentMOI = Optional.empty();
    protected Optional<Distance> opponentWheelRadius = Optional.empty();
    protected Optional<LinearVelocity> opponentDriveVelocity = Optional.empty();
    protected Optional<Double> opponentDriveCOF = Optional.empty();
    protected Optional<DCMotor> opponentDriveMotor = Optional.empty();
    protected Optional<Double> opponentDriveCurrentLimit = Optional.empty();
    protected Optional<Integer> opponentNumDriveMotors = Optional.empty();
    protected Optional<Distance> opponentTrackWidth = Optional.empty();
    protected Optional<Object> joystick = Optional.empty();

    // Static settings
    protected Double joystickDeadzone = 0.1;
    protected Distance driveToPoseCollectTolerance = Inches.of(8);
    protected Distance driveToPoseScoreTolerance = Inches.of(3);
    protected String robotName = "Smart Opponent";

    // Obstacle avoidance
    private static final double OBSTACLE_BUFFER = 0.3;
    protected boolean isColliding = false;

    /**
     * @param id
     * @param alliance
     */
    public void setupOpponent(OpponentManager manager, DriverStation.Alliance alliance, int id) {
        this.id = Optional.of(id);
        this.alliance = Optional.of(alliance);
        this.startPose = Optional.of(manager.getStartingPose(alliance, id));
        this.queeningPose = Optional.of(manager.getQueeningPose(alliance, id));
        this.driveConfig = Optional.of(DriveTrainSimulationConfig.Default());
        this.simulation = Optional.of(
                new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(
                        driveConfig.get(),
                        queeningPose.get()
                )));
        this.isJoystick = Optional.of(new Trigger(() -> currentState.get() == States.JOYSTICK));
        SimulatedArena.getInstance().addDriveTrainSimulation(
                this.simulation.get().getDriveTrainSimulation());
        this.posePublisher = Optional.of(
                NetworkTableInstance.getDefault()
                        .getStructTopic("SmartDashboard/MapleSim/SimulatedRobots/Poses/ "
                                + (alliance.equals(DriverStation.Alliance.Red) ? "Red Alliance " : "Blue Alliance ")
                                + robotName + " " + id + " Pose", Pose2d.struct).publish());
        this.statePublisher = Optional.of(
                NetworkTableInstance.getDefault()
                        .getStringTopic("SmartDashboard/MapleSim/SimulatedRobots/States/ "
                                + (alliance.equals(DriverStation.Alliance.Red) ? "Red Alliance " : "Blue Alliance ")
                                + robotName + " " + id + " Current State").publish());
        this.target = Optional.of(Pose2d.kZero);
        setState(States.STANDBY);
        buildBehaviorChooser(id, alliance);
        manager.registerOpponent(this, alliance);
    }

    /**
     * @param newState
     */
    public void setState(States newState) {
        currentState = Optional.of(newState);
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
            Optional<States> thisState = currentState; // States change during the state command.
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
    public void buildBehaviorChooser(int id, DriverStation.Alliance alliance) {
        if (simulation.isPresent() && queeningPose.isPresent()) {
            this.behaviorChooser = Optional.of(new SendableChooser<>());

            // Option to disable the robot
            behaviorChooser.get().setDefaultOption("Disable", Commands.runOnce(() -> setState(States.STANDBY))
                    .andThen(standbyCommand()));

            // Option to auto-cycle random
            behaviorChooser.get().addOption(
                    "Smart Cycle", Commands.runOnce(() -> setState(States.STARTING)));

            // Schedule the command when another behavior is selected
            behaviorChooser.get().onChange((Command::schedule));

            // Schedule the selected command when teleop starts
            RobotModeTriggers.teleop()
                    .onTrue(Commands.runOnce(() -> behaviorChooser.get().getSelected().schedule()));

            // Disable the robot when the user robot is disabled
            RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> setState(States.STANDBY))
                    .andThen(standbyCommand()));

            SmartDashboard.putData("MapleSim/SimulatedRobots/Behaviors/ "
                    + (alliance.equals(DriverStation.Alliance.Red) ? "Red Alliance " : "Blue Alliance ")
                    + robotName + " " + id + " Behavior", behaviorChooser.get());
        } else {
            DriverStation.reportWarning("No simulation found", false);
        }
    }

    @Override
    public void simulationPeriodic() {
        currentState.ifPresent(state -> {
            if (!commandInProgress && (previousState.isEmpty() || state != previousState.get())) {
                runStateCommand(state);
                previousState = Optional.of(state);
                commandInProgress = true;
            }
        });
        simulation.ifPresent(
                simulation -> posePublisher.ifPresent(
                        posePublisher -> posePublisher.set(simulation.getActualPoseInSimulationWorld())));
        currentState.ifPresent(
                state -> statePublisher.ifPresent(
                        statePublisher -> statePublisher.set(state.toString())));
    }

    /**
     *
     */
    public Command standbyCommand() {
        if (simulation.isPresent() && queeningPose.isPresent()) {
            return Commands.runOnce(() -> simulation.get().runChassisSpeeds(
                            new ChassisSpeeds(), new Translation2d(), false, false), this)
                    .andThen(Commands.runOnce(() -> simulation.get().setSimulationWorldPose(queeningPose.get()), this))
                    .ignoringDisable(true);
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }

    /**
     *
     */
    public Command startingCommand() {
        if (simulation.isPresent() && startPose.isPresent()) {
            return Commands.runOnce(() -> simulation.get().setSimulationWorldPose(startPose.get()))
                    .andThen(Commands.runOnce(() -> simulation.get().runChassisSpeeds(
                            new ChassisSpeeds(), new Translation2d(), false, false), this))
                    .ignoringDisable(true)
                    .andThen(Commands.waitSeconds(1))
                    .andThen(() -> setState(States.COLLECT));
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }

    /**
     *
     */
    public Command collectCommand() {
        if (simulation.isPresent() && currentState.isPresent() && manager.isPresent() && alliance.isPresent()) {
            return (pathfindCommand(manager.get().getNextCollectTarget(alliance.get()), driveToPoseCollectTolerance).withTimeout(7)
                    .andThen(Commands.waitSeconds(0.5)))
                    .andThen(() -> setState(States.SCORE));
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }

    /**
     *
     */
    public Command scoreCommand() {
        if (simulation.isPresent() && currentState.isPresent()) {
            this.target = Optional.of(manager.get().getNextCollectTarget(alliance.get()));
            return
                    (pathfindCommand(target.get(), driveToPoseScoreTolerance).withTimeout(7)
                            .andThen(Commands.waitSeconds(0.5)))
                            .until(() -> isColliding)
                            .andThen(() -> setState(States.COLLECT));
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }

    /**
     * @return
     */
    public Command joystickCommand() {
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
        behaviorChooser.ifPresent(
                chooser -> chooser.addOption("Joystick Drive", Commands.runOnce(() -> setState(States.JOYSTICK))));
        return this;
    }

    /**
     *
     */
    public Command joystickDrive() {
        if (simulation.isPresent() && startPose.isPresent() && currentState.isPresent() && joystick.isPresent()
                && joystick.get() instanceof CommandXboxController controller) {
            return Commands.runOnce(() -> simulation.get().setSimulationWorldPose(startPose.get()))
                    .andThen(Commands.runOnce(() -> simulation.get().runChassisSpeeds(
                            new ChassisSpeeds(), new Translation2d(), false, false), this))
                    .ignoringDisable(true)
                    .andThen(Commands.waitSeconds(1))
                    .andThen(Commands.run(() -> {
                        // Calculate field-centric speed from driverstation speed
                        final ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                new ChassisSpeeds(
                                        MathUtil.applyDeadband(-controller.getLeftY(), joystickDeadzone) * simulation.get().maxLinearVelocity().in(MetersPerSecond),
                                        MathUtil.applyDeadband(-controller.getLeftX(), joystickDeadzone) * simulation.get().maxLinearVelocity().in(MetersPerSecond),
                                        MathUtil.applyDeadband(-controller.getRightX(), joystickDeadzone) * simulation.get().maxAngularVelocity().in(RadiansPerSecond)),
                                DriverStation.getAlliance().equals(alliance) ?
                                        FieldMirroringUtils.getCurrentAllianceDriverStationFacing() :
                                        FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                                                .plus(Rotation2d.k180deg));
                        // Run the field-centric speed
                        simulation.get().runChassisSpeeds(speeds, new Translation2d(), false, false);
                    }, this));
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }

    /**
     * This should probably just cycle between 2 poses
     *
     * @return
     */
    public Command defendCommand() {
        return Commands.runOnce(() -> DriverStation.reportWarning("No defend state implemented", false), this);
    }

    /**
     * Advanced pathfinding with obstacle avoidance
     */
    public Command pathfindCommand(Pose2d targetPose, Distance tolerance) {
        if (!simulation.isPresent()) {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }

        return Commands.run(() -> {
            Pose2d robotPose = simulation.get().getActualPoseInSimulationWorld();
            double rX = robotPose.getX(), rY = robotPose.getY();
            double tX = targetPose.getX(), tY = targetPose.getY();
            Pose2d driveTo = targetPose;

            // Inline collision check with obstacle avoidance
            if (obstacles.isPresent()) {
                for (OpponentManager.ObstacleRect obs : obstacles.get()) {
                    if (obs.intersectsLine(rX, rY, tX, tY, OBSTACLE_BUFFER)) {
                        // Find closest obstacle using squared distance (no sqrt)
                        double minDistSq = Double.MAX_VALUE;
                        OpponentManager.ObstacleRect closest = null;
                        for (OpponentManager.ObstacleRect o : obstacles.get()) {
                            double dx = rX - o.centerX, dy = rY - o.centerY;
                            double distSq = dx * dx + dy * dy;
                            if (distSq < minDistSq) {
                                minDistSq = distSq;
                                closest = o;
                            }
                        }
                        if (closest != null) {
                            double offset = (rX < closest.centerX) ? -OBSTACLE_BUFFER * 2 : OBSTACLE_BUFFER * 2;
                            driveTo = new Pose2d(closest.centerX + offset, closest.centerY, robotPose.getRotation());
                        }
                        break;
                    }
                }
            }

            // Calculate chassis speeds toward target
            double dx = driveTo.getX() - rX;
            double dy = driveTo.getY() - rY;
            double distSq = dx * dx + dy * dy;
            double distTolerance = tolerance.in(Meters);

            if (distSq > distTolerance * distTolerance) {
                double distance = Math.sqrt(distSq);
                double maxVel = simulation.get().maxLinearVelocity().in(MetersPerSecond);
                double vx = (dx / distance) * maxVel;
                double vy = (dy / distance) * maxVel;
                double omega = driveTo.getRotation().minus(robotPose.getRotation()).getRadians() * 2;
                simulation.get().runChassisSpeeds(new ChassisSpeeds(vx, vy, omega), new Translation2d(), false, false);
            } else {
                simulation.get().runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false);
            }
        }, this).until(() -> simulation.isPresent() &&
                simulation.get().getActualPoseInSimulationWorld().getTranslation()
                        .getDistance(targetPose.getTranslation()) <= tolerance.in(Meters));
    }

    public Pose2d getTarget() {
        return target.orElse(Pose2d.kZero);
    }

    public Pose2d getPose() {
        Pose2d pose = new Pose2d();
        if (simulation.isPresent()) {
            pose = simulation.get().getActualPoseInSimulationWorld();
        }
        return pose;
    }

    /**
     * returns false if simulation isn't present
     */
    public boolean nearPose(Pose2d pose, Distance maxDistance) {
        if (simulation.isPresent()) {
            Translation2d robotTranslation = simulation.get().getActualPoseInSimulationWorld().getTranslation();
            Translation2d goalTranslation = pose.getTranslation();
            double distance = robotTranslation.getDistance(goalTranslation);
            return distance <= maxDistance.in(Meters);
        } else {
            return false;
        }
    }

    /**
     *
     */
    public Command coralFeedShotCommand() {
        // Setup to match the 2025 kitbot
        // Coral Settings
        Distance shootHeight = Meters.of(0.85);
        LinearVelocity shootSpeed = MetersPerSecond.of(1);
        Angle shootAngle = Degrees.of(-15);
        Translation2d shootOnBotPosition = new Translation2d(
                0.5,
                0);

        return runOnce(() ->
        {
            simulation.ifPresent(
                    simulation ->
                    {
                        SimulatedArena.getInstance()
                                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                                        // Obtain robot position from drive simulation
                                        simulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getTranslation(),
                                        // The scoring mechanism is installed at (x, y) (meters) on the robot
                                        shootOnBotPosition,
                                        // Obtain robot speed from drive simulation
                                        simulation.getDriveTrainSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                        // Obtain robot facing from drive simulation
                                        simulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getRotation(),
                                        // The height at which the coral is ejected
                                        shootHeight,
                                        // The initial speed of the coral
                                        shootSpeed,
                                        // The coral is ejected at a 35-degree slope
                                        shootAngle));
                    });
        });
    }

    /**
     *
     */
    public void coralFeedShot() {
        // Setup to match the 2025 kitbot
        // Coral Settings
        Distance shootHeight = Meters.of(0.85);
        LinearVelocity shootSpeed = MetersPerSecond.of(1);
        Angle shootAngle = Degrees.of(-15);
        Translation2d shootOnBotPosition = new Translation2d(
                0.5,
                0);

        Commands.runOnce(() -> {
            simulation.ifPresent(
                    simulation ->
                    {
                        SimulatedArena.getInstance()
                                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                                        // Obtain robot position from drive simulation
                                        simulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getTranslation(),
                                        // The scoring mechanism is installed at (x, y) (meters) on the robot
                                        shootOnBotPosition,
                                        // Obtain robot speed from drive simulation
                                        simulation.getDriveTrainSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                        // Obtain robot facing from drive simulation
                                        simulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getRotation(),
                                        // The height at which the coral is ejected
                                        shootHeight,
                                        // The initial speed of the coral
                                        shootSpeed,
                                        // The coral is ejected at a 35-degree slope
                                        shootAngle));
                    });
        }, this);
    }

    /**
     *
     * @return
     */
    public Command algaeFeedShotCommand() {
        // Algae settings
        Distance shootHeight = Meters.of(3);
        LinearVelocity shootSpeed = MetersPerSecond.of(.9);
        Angle shootAngle = Degrees.of(35);
        Translation2d shootOnBotPosition = new Translation2d(
                Inches.of(6).in(Meters),
                0);

        return runOnce(() ->
        {
            simulation.ifPresent(
                    simulation -> {
                        GamePieceProjectile algae = new ReefscapeAlgaeOnFly(
                                // Obtain robot position from drive simulation
                                simulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getTranslation(),
                                // The scoring mechanism is installed at (x, y) (meters) on the robot
                                shootOnBotPosition,
                                // Obtain robot speed from drive simulation
                                simulation.getDriveTrainSimulation().getDriveTrainSimulatedChassisSpeedsRobotRelative(),
                                // Obtain robot facing from drive simulation
                                simulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getRotation(),
                                // The height at which the coral is ejected
                                shootHeight,
                                // The initial speed of the coral
                                shootSpeed,
                                // The coral is ejected at a 35-degree slope
                                shootAngle);
                        SimulatedArena.getInstance()
                                .addGamePieceProjectile(algae);
                    });
        });
    }

    /**
     *
     * @return
     */
    public void algaeFeedShot() {
        // Algae settings
        Distance shootHeight = Meters.of(3);
        LinearVelocity shootSpeed = MetersPerSecond.of(.9);
        Angle shootAngle = Degrees.of(35);
        Translation2d shootOnBotPosition = new Translation2d(
                Inches.of(6).in(Meters),
                0);

        simulation.ifPresent(
                simulation -> {
                    GamePieceProjectile algae = new ReefscapeAlgaeOnFly(
                            // Obtain robot position from drive simulation
                            simulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getTranslation(),
                            // The scoring mechanism is installed at (x, y) (meters) on the robot
                            shootOnBotPosition,
                            // Obtain robot speed from drive simulation
                            simulation.getDriveTrainSimulation().getDriveTrainSimulatedChassisSpeedsRobotRelative(),
                            // Obtain robot facing from drive simulation
                            simulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getRotation(),
                            // The height at which the coral is ejected
                            shootHeight,
                            // The initial speed of the coral
                            shootSpeed,
                            // The coral is ejected at a 35-degree slope
                            shootAngle);
                    SimulatedArena.getInstance()
                            .addGamePieceProjectile(algae);
                });
    }

    /**
     * @param pose
     * @return
     */
    public Pose2d ifShouldFlip(Pose2d pose) {
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                return pose;
            } else {
                return new Pose2d(
                        FieldMirroringUtils.flip(pose.getTranslation()),
                        FieldMirroringUtils.flip(pose.getRotation()));
            }
        } else {
            return pose;
        }
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
