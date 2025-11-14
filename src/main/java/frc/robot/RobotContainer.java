// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import yams.mechanisms.swerve.utility.SwerveInputStream;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...'
    private final CommandXboxController xboxController = new CommandXboxController(0);
    private final SwerveSubsystem drive = new SwerveSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
      // Sets up the input stream for the swerve drive
      SwerveInputStream stream = drive.getChassisSpeedsSupplier(
              () -> xboxController.getLeftX() * -1,
              () -> xboxController.getLeftY() * -1,
              () -> xboxController.getRightX() * -1);
      // Sets the default command for the swerve drive
      drive.setDefaultCommand(drive.driveRobotRelative(stream));
      // Binds a toggle for the robot relative and alliance relative controls
      xboxController.start().toggleOnTrue(Commands.runOnce(() -> stream.withRobotRelative().withAllianceRelativeControl(() -> false)))
              .toggleOnFalse(Commands.runOnce(() -> stream.withAllianceRelativeControl().withRobotRelative(() -> false)));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
