package org.ironmaple.simulation.opponentsim;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

import java.util.HashMap;
import java.util.Map;

public class OpponentManipulatorSim extends SubsystemBase {
    private final Map<String, IntakeSimulation> intakeSimulations;
    private final Map<String, GamePieceProjectile> projectileSimulations;

    /**
     * Creates a new manipulator simulation.
     */
    public OpponentManipulatorSim()
    {
        this.intakeSimulations = new HashMap<>();
        this.projectileSimulations = new HashMap<>();
    }

    /**
     * Adds an intake simulation to the manipulator simulation.
     *
     * @param intakeName The name of the intake simulation.
     * @param intakeSimulation The simulation to add.
     * @return this, for chaining.
     */
    public OpponentManipulatorSim addIntakeSimulation(String intakeName, IntakeSimulation intakeSimulation)
    {
        this.intakeSimulations.put(intakeName, intakeSimulation);
        return this;
    }

    /**
     * Adds a projectile simulation to the manipulator simulation.
     *
     * @param projectileName The name of the projectile simulation.
     * @param projectileSimulation The simulation to add.
     * @return this, for chaining.
     */
    public OpponentManipulatorSim addProjectileSimulation(String projectileName, GamePieceProjectile projectileSimulation)
    {
        this.projectileSimulations.put(projectileName, projectileSimulation);
        return this;
    }

    /**
     * Gets an intake simulation from the manipulator simulation.
     *
     * @param intakeName The name of the intake simulation.
     * @return The simulation.
     */
    public IntakeSimulation getIntakeSimulation(String intakeName)
    {
        return this.intakeSimulations.get(intakeName);
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
     * Runs the intake, stops running the intake when the command ends.
     *
     * @param intakeName
     * @return
     */
    public Command intake(String intakeName)
    {
        return runEnd(() -> getIntakeSimulation(intakeName).startIntake(),
                () -> getIntakeSimulation(intakeName).stopIntake());
    }

    /**
     * Runs intake(String) until the game piece count in the intake goes up.
     *
     * @param intakeName
     * @return
     */
    public Command intakeUntilCollected(String intakeName)
    {
        final int count = getIntakeSimulation(intakeName).getGamePiecesAmount();
        return intake(intakeName)
                .until(() -> count > getIntakeSimulation(intakeName).getGamePiecesAmount());
    }

    /**
     * Adds a projectile to the simulation.
     *
     * @param projectileName The name of the projectile simulation.
     * @return a command to add the game piece projectile to the simulation.
     */
    public Command score(String projectileName)
    {
        return runOnce(() -> SimulatedArena.getInstance().addGamePieceProjectile(getProjectileSimulation(projectileName)));
    }

    /**
     * A command that only calls score(String) when there is greater than zero(>0) {@link org.ironmaple.simulation.gamepieces.GamePiece} in the intake.
     *
     * @param intakeName
     * @param projectileName
     * @return
     */
    public Command scoreWithIntake(String intakeName, String projectileName)
    {
        return runOnce(() -> {
            if (getIntakeSimulation(intakeName).getGamePiecesAmount() > 0)
            {
                score(projectileName);
            }
        });
    }
}
