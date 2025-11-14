package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.ironmaple.simulation.opponentsim.OpponentManager;
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
  private final double OBSTACLE_BUFFER = 0.3;
  private final List<OpponentManager.ObstacleRect> obstacles = new ArrayList<>();

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
     * Advanced pathfinding with obstacle avoidance using potential fields
     */
    public Command pathfindCommand(Pose2d targetPose, Distance tolerance) {
        return Commands.run(() -> {
            Pose2d robotPose = drive.getPose();
            double rX = robotPose.getX(), rY = robotPose.getY();
            double tX = targetPose.getX(), tY = targetPose.getY();
            double distTolerance = tolerance.in(Meters);

            // Attraction force toward target
            double attractX = tX - rX;
            double attractY = tY - rY;
            double attractDist = Math.hypot(attractX, attractY);
            double attractWeight = 1.0;

            if (attractDist > 0.01) {
                attractX = (attractX / attractDist) * attractWeight;
                attractY = (attractY / attractDist) * attractWeight;
            } else {
                attractX = attractY = 0;
            }

            // Repulsion forces from obstacles
            double repelX = 0, repelY = 0;
            for (OpponentManager.ObstacleRect obs : obstacles) {
                double obsDist = Math.hypot(rX - obs.centerX, rY - obs.centerY);
                double repelRadius = Math.sqrt(obs.width * obs.width + obs.height * obs.height) + OBSTACLE_BUFFER;

                if (obsDist < repelRadius && obsDist > 0.01) {
                    double repelStrength = repelRadius / (obsDist + 0.1);
                    double angle = Math.atan2(rY - obs.centerY, rX - obs.centerX);
                    repelX += Math.cos(angle) * repelStrength;
                    repelY += Math.sin(angle) * repelStrength;
                }
            }

            // Combine forces with obstacle weight higher to ensure avoidance
            double totalX = attractX + repelX * 0.8;
            double totalY = attractY + repelY * 0.8;
            double totalMag = Math.hypot(totalX, totalY);

            if (totalMag > 0.01) {
                double maxVel = maximumChassisSpeedsLinearVelocity.in(MetersPerSecond);
                double vx = (totalX / totalMag) * maxVel;
                double vy = (totalY / totalMag) * maxVel;
                double omega = targetPose.getRotation().minus(robotPose.getRotation()).getRadians() * 2;
                drive.drive(() -> new ChassisSpeeds(vx, vy, omega));
            } else {
                drive.drive(ChassisSpeeds::new);
            }
        }, this).until(() -> drive.getPose().getTranslation()
                .getDistance(targetPose.getTranslation()) <= tolerance.in(Meters));
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
  }
}

