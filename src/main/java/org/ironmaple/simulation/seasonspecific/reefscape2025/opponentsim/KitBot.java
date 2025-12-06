package org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.opponentsim.SmartOpponent;
import org.ironmaple.simulation.opponentsim.configs.SmartOpponentConfig;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import static edu.wpi.first.units.Units.*;

public class KitBot extends SmartOpponent {

    public KitBot(String name, DriverStation.Alliance alliance) {
        /// All Options should be set in the constructor.
        super(new SmartOpponentConfig()
                .withName(name)
                .withAlliance(alliance)
                .withStartingPose(new Pose2d(1.6, 6, new Rotation2d()))
                .withQueeningPose(new Pose2d(-6, 0, new Rotation2d()))
                .withChassisConfig(SmartOpponentConfig.ChassisConfig.Presets.SimpleSquareChassis.getConfig())
                .addScoringPose("Reef", "SouthLeft", new Pose2d( // South Center Face
                        Units.inchesToMeters(144.003-28),
                        Units.inchesToMeters(158.500),
                        Rotation2d.fromDegrees(0))
                        .plus(new Transform2d(
                                Units.inchesToMeters(-28), // Offset away from the reef.
                                Units.inchesToMeters(13 / 2.0), // Offset to the left branch.
                                Rotation2d.kZero)))
                .addCollectingPose("Station", "LeftCenter", new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(291.176),
                        Rotation2d.fromDegrees(125.989)))
                .withAutoEnable());
        this.manipulatorSim = new ManipulatorSim()
                .addIntakeSimulation("Intake",
                        IntakeSimulation.InTheFrameIntake(
                                "Coral",
                                config.chassis.driveTrainSim.getDriveTrainSimulation(),
                                Inches.of(20),
                                IntakeSimulation.IntakeSide.BACK,
                                1))
                .addProjectileSimulation("Coral",
                        new ReefscapeCoralOnFly(
                                drivetrainSim.getActualPoseInSimulationWorld().getTranslation(),
                                new Translation2d(Inches.of(0), Inches.of(20)),
                                drivetrainSim.getActualSpeedsFieldRelative(),
                                drivetrainSim.getActualPoseInSimulationWorld().getRotation(),
                                Inches.of(12),
                                InchesPerSecond.of(12),
                                Degrees.of(-15)));
    }

    /**
     * The collect state to run.
     *
     * @return a runnable that runs the state.
     */
    @Override
    protected Command collectState() {
        return pathfind(getRandomFromMap(config.getCollectingMap())).withTimeout(10)
                .andThen(() -> manipulatorSim.getIntakeSimulation("Intake").startIntake()).withTimeout(0.5)
                .finallyDo(() -> manipulatorSim.getIntakeSimulation("Intake").stopIntake()).withTimeout(0.1)
                .finallyDo(() -> setState("Score"));
    }

    /**
     * The score state to run.
     *
     * @return a runnable that runs the state.
     */
    @Override
    protected Command scoreState() {
        return pathfind(getRandomFromMap(config.getScoringMap())).withTimeout(10)
                .andThen(() -> manipulatorSim.feedShot("Coral"))
                .finallyDo(() -> setState("Collect"));
    }
}