// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim.KitBot;
import org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim.ReefscapeOpponentManager;
import yams.mechanisms.swerve.utility.SwerveInputStream;

import static edu.wpi.first.units.Units.Inches;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...'
    private final CommandXboxController xboxController = new CommandXboxController(0);
    private final SwerveSubsystem drive = new SwerveSubsystem();

  public RobotContainer() {
    configureBindings();


  }

  private void configureBindings() {
      SimulatedArena.getInstance().resetFieldForAuto();
      ReefscapeOpponentManager manager = new ReefscapeOpponentManager();
      new KitBot(manager, DriverStation.Alliance.Blue, 1);
      new KitBot(manager, DriverStation.Alliance.Red, 2);

      // Sets up the input stream for the swerve drive
      SwerveInputStream stream = drive.getChassisSpeedsSupplier(
              xboxController::getLeftY,
              xboxController::getLeftX,
              () -> xboxController.getRightX() * -1);
      // Sets the default command for the swerve drive
      drive.setDefaultCommand(drive.driveRobotRelative(stream));
      // Binds a toggle for the robot relative and alliance relative controls
      xboxController.start().toggleOnTrue(Commands.runOnce(() -> stream.withRobotRelative().withAllianceRelativeControl(() -> false)))
              .toggleOnFalse(Commands.runOnce(() -> stream.withAllianceRelativeControl().withRobotRelative(() -> false)));

      xboxController.back().onTrue(drive.resetRobotPose());
      // Binds a button to run the pathfind command
      xboxController.a().whileTrue(drive.driveToPose(new Pose2d(new Translation2d(3, 3), Rotation2d.kZero)));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
