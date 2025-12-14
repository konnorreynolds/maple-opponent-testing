package org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.opponentsim.SmartOpponent;
import org.ironmaple.simulation.opponentsim.SmartOpponentConfig;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import static edu.wpi.first.units.Units.*;

public class KitBot extends SmartOpponent {
    private final Transform2d poseOffset = new Transform2d(new Translation2d(Inches.of(-12), Inches.of(0)), Rotation2d.kZero);
    public KitBot(String name, DriverStation.Alliance alliance) {
        /// All Options should be set in the constructor.
        super(new SmartOpponentConfig()
                .withName(name)
                .withAlliance(alliance)
                .withStartingPose(ReefscapeOpponentPoses.getInitialPose(alliance))
                .withQueeningPose(ReefscapeOpponentPoses.getQueeningPose())
                .withChassisConfig(SmartOpponentConfig.ChassisConfig.Presets.SimpleSquareChassis.getConfig())
                .addScoringMap(ReefscapeOpponentPoses.getRawScoringMap())
                .removeScoringPoseType("Barge")
                .addCollectingMap(ReefscapeOpponentPoses.getRawCollectingMap())
                .withAutoEnable());
        this.manipulatorSim
                .addIntakeSimulation("Intake",
                        IntakeSimulation.InTheFrameIntake(
                                "Coral",
                                drivetrainSim.getDriveTrainSimulation(),
                                Inches.of(20),
                                IntakeSimulation.IntakeSide.BACK,
                                1))
                .addProjectileSimulation("Coral",
                        () -> new ReefscapeCoralOnFly(
                                drivetrainSim.getActualPoseInSimulationWorld().getTranslation(),
                                new Translation2d(Inches.of(0), Inches.of(-18)), // Shooter on bot
                                drivetrainSim.getActualSpeedsFieldRelative()
                                        .plus(ChassisSpeeds.fromRobotRelativeSpeeds(
                                                new ChassisSpeeds(1, 0, 0), // Added chassis speeds to change coral velocity,
                                                drivetrainSim.getActualPoseInSimulationWorld().getRotation())),                 // this is because we want a horizontal score
                                drivetrainSim.getActualPoseInSimulationWorld().getRotation()
                                        .rotateBy(Rotation2d.kCCW_90deg), // Rotated by 90 degrees for horizontal shooter.
                                Inches.of(30), // Shooter Height
                                MetersPerSecond.of(0), // Initial piece speed
                                Degrees.of(0))); // Shooter angle
    }

    /**
     * The collect state to run.
     *
     * @return a runnable that runs the state.
     */
    @Override
    protected Command collectState() {
        return pathfind(getRandomFromMap(config.getCollectingMap()), Seconds.of(4))
                .andThen(manipulatorSim.intake("Intake")
                        .withDeadline(Commands.waitSeconds(1)))
                .finallyDo(() -> setState("Score"))
                .withTimeout(10);
    }

    /**
     * The score state to run.
     *
     * @return a runnable that runs the state.
     */
    @Override
    protected Command scoreState() {
        return pathfind(getRandomFromMap(config.getScoringMap()), Seconds.of(6))
                .andThen(Commands.waitSeconds(0.2))
                .andThen(manipulatorSim.score("Coral"))
                .andThen(Commands.waitSeconds(0.5))
                .finallyDo(() -> setState("Collect"))
                .withTimeout(10);
    }

    public KitBot withXboxController(CommandXboxController xboxController) {
        config.withJoystick(xboxController);
        config.addState("Joystick", this::joystickState);
        config.addBehavior("Player", startingState("Joystick").andThen(startingState("Joystick").ignoringDisable(true)));
        config.updateBehaviorChooser();
        return this;
    }

    /**
     * The joystick state to run.
     * Should be inaccessible when not set.
     *
     * @return the joystick state to run.
     */
    private Command joystickState() {
        final CommandXboxController xbox = ((CommandXboxController) config.getJoystick());
        return drive(() ->
                        new ChassisSpeeds(
                                MathUtil.applyDeadband(xbox.getLeftY() * -config.chassis.maxLinearVelocity.in(MetersPerSecond), config.joystickdeadband),
                                MathUtil.applyDeadband(xbox.getLeftX() * -config.chassis.maxLinearVelocity.in(MetersPerSecond), config.joystickdeadband),
                                MathUtil.applyDeadband(xbox.getRightX() * -config.chassis.maxAngularVelocity.in(RadiansPerSecond), config.joystickdeadband)),
                        false);
    }
}