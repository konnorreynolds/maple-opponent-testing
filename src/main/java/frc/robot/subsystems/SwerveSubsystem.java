package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Meters;

public class SwerveSubsystem extends SubsystemBase
{
    private final StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/GamePieces")
            .getStructArrayTopic("Coral Array",
                    Pose3d.struct)
            .publish();
    private final StructArrayPublisher<Pose3d> algaePoses = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/GamePieces")
            .getStructArrayTopic("Algae Array",
                    Pose3d.struct)
            .publish();

  private final SwerveDrive drive;
  private final Field2d     field = new Field2d();

  private AngularVelocity maximumChassisSpeedsAngularVelocity = DegreesPerSecond.of(720);
  private LinearVelocity  maximumChassisSpeedsLinearVelocity  = MetersPerSecond.of(4);

  /**
   * Get a {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds based on "standard" swerve drive
   * controls.
   *
   * @param translationXScalar Translation in the X direction from [-1,1]
   * @param translationYScalar Translation in the Y direction from [-1,1]
   * @param rotationScalar     Rotation speed from [-1,1]
   * @return {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds.
   */
  public SwerveInputStream getChassisSpeedsSupplier(DoubleSupplier translationXScalar,
                                                    DoubleSupplier translationYScalar,
                                                    DoubleSupplier rotationScalar)
  {
    return new SwerveInputStream(drive, translationXScalar, translationYScalar, rotationScalar)
        .withMaximumAngularVelocity(maximumChassisSpeedsAngularVelocity)
        .withMaximumLinearVelocity(maximumChassisSpeedsLinearVelocity)
        .withCubeRotationControllerAxis()
        .withCubeTranslationControllerAxis()
        .withAllianceRelativeControl()
        .withDeadband(0.01);
  }

  /**
   * Get a {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds based on "standard" swerve drive
   * controls.
   *
   * @param translationXScalar Translation in the X direction from [-1,1]
   * @param translationYScalar Translation in the Y direction from [-1,1]
   * @param rotationScalar     Rotation speed from [-1,1]
   * @return {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds.
   */
  public Supplier<ChassisSpeeds> getSimpleChassisSpeeds(DoubleSupplier translationXScalar,
                                                        DoubleSupplier translationYScalar,
                                                        DoubleSupplier rotationScalar)
  {
    return () -> new ChassisSpeeds(maximumChassisSpeedsLinearVelocity.times(translationXScalar.getAsDouble())
                                                                     .in(MetersPerSecond),
                                   maximumChassisSpeedsLinearVelocity.times(translationYScalar.getAsDouble())
                                                                     .in(MetersPerSecond),
                                   maximumChassisSpeedsAngularVelocity.times(rotationScalar.getAsDouble())
                                                                      .in(RadiansPerSecond));
  }

  public SwerveModule createModule(SparkMax drive, SparkMax azimuth, CANcoder absoluteEncoder, String moduleName,
                                   Translation2d location)
  {
    MechanismGearing driveGearing         = new MechanismGearing(GearBox.fromStages("12:1", "2:1"));
    MechanismGearing azimuthGearing       = new MechanismGearing(GearBox.fromStages("21:1"));
    PIDController    azimuthPIDController = new PIDController(1, 0, 0);
    SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(4))
        .withClosedLoopController(50, 0, 4)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withGearing(driveGearing)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("driveMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
        .withClosedLoopController(50, 0, 4)
        .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
        .withGearing(azimuthGearing)
        .withStatorCurrentLimit(Amps.of(20))
        .withTelemetry("angleMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    SmartMotorController driveSMC   = new SparkWrapper(drive, DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthSMC = new SparkWrapper(azimuth, DCMotor.getNEO(1), azimuthCfg);
    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
        .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
        .withTelemetry(moduleName, SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withLocation(location)
        .withOptimization(true);
    return new SwerveModule(moduleConfig);
  }

  public SwerveSubsystem()
  {
    Pigeon2 gyro = new Pigeon2(14);
    var fl = createModule(new SparkMax(1, MotorType.kBrushless),
                          new SparkMax(2, MotorType.kBrushless),
                          new CANcoder(3),
                          "frontleft",
                          new Translation2d(Inches.of(24), Inches.of(24)));
    var fr = createModule(new SparkMax(4, MotorType.kBrushless),
                          new SparkMax(5, MotorType.kBrushless),
                          new CANcoder(6),
                          "frontright",
                          new Translation2d(Inches.of(24), Inches.of(-24)));
    var bl = createModule(new SparkMax(7, MotorType.kBrushless),
                          new SparkMax(8, MotorType.kBrushless),
                          new CANcoder(9),
                          "backleft",
                          new Translation2d(Inches.of(-24), Inches.of(24)));
    var br = createModule(new SparkMax(10, MotorType.kBrushless),
                          new SparkMax(11, MotorType.kBrushless),
                          new CANcoder(12),
                          "backright",
                          new Translation2d(Inches.of(-24), Inches.of(-24)));
    SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
        .withGyro(gyro.getYaw().asSupplier())
        .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));
    drive = new SwerveDrive(config);

    SmartDashboard.putData("Field", field);
  }

  public Command driveToPose(Pose2d pose)
  {
      return drive.driveToPose(pose);
  }

  /**
   * Drive the {@link SwerveDrive} object with robot relative chassis speeds.
   *
   * @param speedsSupplier Robot relative {@link ChassisSpeeds}.
   * @return {@link Command} to run the drive.
   */
  public Command drive(Supplier<ChassisSpeeds> speedsSupplier)
  {
    return drive.drive(speedsSupplier);
  }

  public Command setRobotRelativeChassisSpeeds(ChassisSpeeds speeds)
  {
    return run(() -> drive.setRobotRelativeChassisSpeeds(speeds));
  }

    /**
     * Calculate ChassisSpeeds to drive to goal while avoiding obstacles.
     *
     * Uses Potential Fields for obstacle avoidance + ProfiledPIDControllers for smooth motion.
     *
     * Setup your controllers once in your class:
     *   private final ProfiledPIDController translationController = new ProfiledPIDController(
     *       2.0, 0.0, 0.0,  // kP, kI, kD
     *       new TrapezoidProfile.Constraints(3.0, 2.0)  // max velocity, max acceleration (m/s, m/s²)
     *   );
     *
     *   private final ProfiledPIDController rotationController = new ProfiledPIDController(
     *       5.0, 0.0, 0.3,  // kP, kI, kD
     *       new TrapezoidProfile.Constraints(Math.PI, 2 * Math.PI)  // max vel, max accel (rad/s, rad/s²)
     *   );
     *
     * In your constructor:
     *   rotationController.enableContinuousInput(-Math.PI, Math.PI);
     *
     * Usage:
     *   ChassisSpeeds speeds = calculateDriveToPose(
     *       getPose(),
     *       goalPose,
     *       getObstacles(),  // or List.of() for none
     *       translationController,
     *       rotationController
     *   );
     *   drive(speeds);
     *
     * @param currentPose Current robot pose from odometry
     * @param goalPose Target pose to drive to
     * @param obstacles List of obstacle positions in field coordinates (empty if none)
     * @param translationController ProfiledPIDController for translation (manages speed/accel limits)
     * @param rotationController ProfiledPIDController for rotation (manages rotation speed/accel)
     * @return Field-relative ChassisSpeeds
     */
    private ChassisSpeeds calculateDriveToPose(
            Pose2d currentPose,
            Pose2d goalPose,
            List<Translation2d> obstacles,
            ProfiledPIDController translationController,
            ProfiledPIDController rotationController) {

        // ==================== TUNING PARAMETERS ====================

        // Maximum speeds (safety limits - controllers may go slower based on their constraints)
        LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(3.0);
        AngularVelocity MAX_ROTATION_SPEED = RadiansPerSecond.of(Math.PI);

        // Goal attraction strength - how aggressively robot moves toward goal
        // Increase (1.5-2.5) if robot is too cautious
        // Decrease (0.5-0.8) if robot overshoots or is too aggressive
        double ATTRACTIVE_GAIN = 1.0;

        // Obstacle repulsion strength - how strongly robot avoids obstacles
        // Increase (3.0-5.0) if robot gets too close to obstacles
        // Decrease (1.5-2.0) if robot avoids obstacles too early/widely
        double REPULSIVE_GAIN = 2.5;

        // Distance at which obstacles start affecting the robot
        // Increase (2.0-3.0 m) to give obstacles wider berth
        // Decrease (1.0-1.2 m) to only avoid very close obstacles
        Distance OBSTACLE_INFLUENCE_RADIUS = Meters.of(1.5);

        // ===========================================================

        Translation2d currentPos = currentPose.getTranslation();
        Translation2d goalPos = goalPose.getTranslation();

        // === STEP 1: Calculate attractive force (pulls toward goal) ===
        Translation2d vectorToGoal = goalPos.minus(currentPos);
        double distanceToGoal = vectorToGoal.getNorm();

        Translation2d attractiveForce = vectorToGoal.times(
                ATTRACTIVE_GAIN / Math.max(distanceToGoal, 0.1)
        );

        // === STEP 2: Calculate repulsive forces (push away from obstacles) ===
        Translation2d totalRepulsiveForce = new Translation2d();

        for (Translation2d obstacle : obstacles) {
            Translation2d vectorFromObstacle = currentPos.minus(obstacle);
            double distanceToObstacle = vectorFromObstacle.getNorm();

            // Only consider obstacles within influence radius
            if (distanceToObstacle < OBSTACLE_INFLUENCE_RADIUS.in(Meters) &&
                    distanceToObstacle > 0.01) {

                // Repulsive force: stronger when closer
                double repulsiveForceMagnitude = REPULSIVE_GAIN *
                        (1.0 / distanceToObstacle - 1.0 / OBSTACLE_INFLUENCE_RADIUS.in(Meters)) /
                        (distanceToObstacle * distanceToObstacle);

                Translation2d repulsiveForceVector = vectorFromObstacle
                        .div(distanceToObstacle)
                        .times(repulsiveForceMagnitude);

                totalRepulsiveForce = totalRepulsiveForce.plus(repulsiveForceVector);
            }
        }

        // === STEP 3: Combine forces to get desired direction ===
        Translation2d combinedForce = attractiveForce.plus(totalRepulsiveForce);

        // === STEP 4: Use translation controller to calculate smooth velocity ===
        // The controller will respect acceleration limits for smooth motion
        double desiredSpeed = translationController.calculate(
                0.0,  // Current "error" (we use 0 since we're using setpoint as target speed)
                Math.min(combinedForce.getNorm(), MAX_LINEAR_SPEED.in(MetersPerSecond))
        );

        // Create velocity vector in the direction of combined force
        Translation2d velocityVector = combinedForce.getNorm() > 0.01
                ? combinedForce.div(combinedForce.getNorm()).times(Math.abs(desiredSpeed))
                : new Translation2d();

        // === STEP 5: Use rotation controller for smooth rotation to goal ===
        double targetAngle = goalPose.getRotation().getRadians();
        double currentAngle = currentPose.getRotation().getRadians();

        double rotationSpeed = rotationController.calculate(currentAngle, targetAngle);

        // Limit rotation speed to maximum
        rotationSpeed = Math.max(
                -MAX_ROTATION_SPEED.in(RadiansPerSecond),
                Math.min(MAX_ROTATION_SPEED.in(RadiansPerSecond), rotationSpeed)
        );

        // === STEP 6: Return field-relative speeds ===
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                velocityVector.getX(),
                velocityVector.getY(),
                rotationSpeed,
                currentPose.getRotation()
        );
    }

  public Command driveRobotRelative(Supplier<ChassisSpeeds> speedsSupplier)
  {
    return drive.drive(speedsSupplier);
  }

  public Command lock()
  {
    return run(drive::lockPose);
  }

  public Command resetRobotPose() {
        return Commands.runOnce(() ->
        drive.resetOdometry(new Pose2d(4, 4, Rotation2d.kZero)));
  }

  @Override
  public void periodic()
  {
    drive.updateTelemetry();
    field.setRobotPose(drive.getPose());
  }

  @Override
  public void simulationPeriodic()
  {
    drive.simIterate();

      // Get the positions of the notes (both on the field and in the air);
      coralPoses.set(SimulatedArena.getInstance()
              .getGamePiecesByType("Coral")
              .toArray(Pose3d[]::new)
      );
      algaePoses.set(SimulatedArena.getInstance()
              .getGamePiecesByType("Algae")
              .toArray(Pose3d[]::new)
      );
  }
}

