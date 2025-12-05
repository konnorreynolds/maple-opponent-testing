package org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.opponentsim.SmartOpponent;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import static edu.wpi.first.units.Units.*;

public class KitBotPro extends SmartOpponent {
    public KitBotPro(ReefscapeOpponentManager manager, DriverStation.Alliance alliance, int id) {
        this.opponentMassKG = Kilograms.of(55);
        this.opponentMOI = 8.0;
        this.opponentWheelRadius = Inches.of(2);
        this.opponentDriveVelocity = MetersPerSecond.of(8.5);
        this.opponentDriveCOF = 1.19;
        this.opponentDriveMotor = DCMotor.getNEO(1).withReduction(8.14);
        this.opponentDriveCurrentLimit = 40.0;
        this.opponentNumDriveMotors = 1;
        this.opponentTrackWidth = Inches.of(23);
        this.robotName = "KitBot Pro";
        setupOpponent(manager, alliance, id);
    }
    // TODO: Make joystick drive accept bindings easily, Event-loop?
    // TODO: Properly Comment

    @Override
    protected Command score() {
        return targetTask.getSecond().equals("Algae") ? algaeFeedShotCommand() : coralFeedShotCommand();
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

        return runOnce(() -> {
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
     * @return
     */
    public Distance getScoreHeight() {
        Distance scoreHeight = switch (((int) Math.round(Math.random() * 3))) {
            case 0 -> Meters.of(.85);
            case 1 -> Meters.of(1);
            case 2 -> Meters.of(1.35);
            case 3 -> Meters.of(2.05);
            default -> Meters.of(.85);
        };
        return scoreHeight;
    }

    /**
     *
     */
    public Command coralFeedShotCommand() {
        // Setup to match the 2025 kitbot
        // Coral Settings
        Distance shootHeight = Meters.of(0.85);
        LinearVelocity shootSpeed = MetersPerSecond.of(1.2);
        Angle shootAngle = Degrees.of(-15);
        Translation2d shootOnBotPosition = new Translation2d(
                0.5,
                0);

        return runOnce(() ->
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
                            getScoreHeight(),
                            // The initial speed of the coral
                            shootSpeed,
                            // The coral is ejected at a 35-degree slope
                            shootAngle));
        });
    }
}
