package org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.opponentsim.SmartOpponent;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.utils.FieldMirroringUtils;

import static edu.wpi.first.units.Units.*;

public class KitBot extends SmartOpponent {
    public KitBot(ReefscapeOpponentManager manager, DriverStation.Alliance alliance, int id) {
        this.opponentMassKG = Kilograms.of(55);
        this.opponentMOI = 8.0;
        this.opponentWheelRadius = Inches.of(2);
        this.opponentDriveVelocity = MetersPerSecond.of(8.5);
        this.opponentDriveCOF = 1.19;
        this.opponentDriveMotor = DCMotor.getNEO(1).withReduction(8.14);
        this.opponentDriveCurrentLimit = 40.0;
        this.opponentNumDriveMotors = 1;
        this.opponentTrackWidth = Inches.of(23);
        this.robotName = "KitBot";

        setupOpponent(manager, alliance, id);
    }

    @Override
    public Command joystickDrive() {
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
                                DriverStation.getAlliance().equals(alliance) ?
                                        FieldMirroringUtils.getCurrentAllianceDriverStationFacing() :
                                        FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                                                .plus(Rotation2d.k180deg));
                        // Run the field-centric speed
                        simulation.runChassisSpeeds(speeds, new Translation2d(), false, false);
                        coralFeedShot();
                    }, this));
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
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

        return runOnce(() -> {
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
        }, this);
    }

    @Override
    protected Command score() {
        return coralFeedShotCommand();
    }

    @Override
    protected void getNextScoreTarget() {
        targetTask = ((ReefscapeOpponentManager) manager).getNextCoralScoreTarget(alliance, id);
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
    }
}
