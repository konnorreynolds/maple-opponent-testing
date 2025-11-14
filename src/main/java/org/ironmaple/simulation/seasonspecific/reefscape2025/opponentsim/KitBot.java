package org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.ironmaple.simulation.opponentsim.OpponentManager;
import org.ironmaple.simulation.opponentsim.SmartOpponent;
import org.ironmaple.utils.FieldMirroringUtils;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class KitBot extends SmartOpponent {
    public KitBot(OpponentManager manager, DriverStation.Alliance alliance, int id) {
        this.opponentMassKG = Optional.of(Kilograms.of(55));
        this.opponentMOI = Optional.of(8.0);
        this.opponentWheelRadius = Optional.of(Inches.of(2));
        this.opponentDriveVelocity = Optional.of(MetersPerSecond.of(8.5));
        this.opponentDriveCOF = Optional.of(1.19);
        this.opponentDriveMotor = Optional.of(DCMotor.getNEO(1)
                .withReduction(8.14));
        this.opponentDriveCurrentLimit = Optional.of(40.0);
        this.opponentNumDriveMotors = Optional.of(1);
        this.opponentTrackWidth = Optional.of(Inches.of(23));
        this.robotName = "KitBot";
        setupOpponent(manager, alliance, id);
    }

    @Override
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
                        coralFeedShot();
                    }, this));
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }
}
