// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule frontLeft =
      new SwerveModule(
          DriveConstants.FRONT_LEFT_DRIVE_MOTOR_PORT,
          DriveConstants.FRONT_LEFT_TURNING_MOTOR_PORT,
          DriveConstants.FRONT_LEFT_DRIVE_ENCODER_PORTS,
          DriveConstants.FRONT_LEFT_TURNING_ENCODER_PORTS,
          DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
          DriveConstants.FRONT_LEFT_TURNING_ENCODER_REVERSED);

  private final SwerveModule rearLeft =
      new SwerveModule(
          DriveConstants.REAR_LEFT_DRIVE_MOTOR_PORT,
          DriveConstants.REAR_LEFT_TURNING_MOTOR_PORT,
          DriveConstants.REAR_LEFT_DRIVE_ENCODER_PORTS,
          DriveConstants.REAR_LEFT_TURNING_ENCODER_PORTS,
          DriveConstants.REAR_LEFT_DRIVE_ENCODER_REVERSED,
          DriveConstants.REAR_LEFT_TURNING_ENCODER_REVERSED);

  private final SwerveModule frontRight =
      new SwerveModule(
          DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_PORT,
          DriveConstants.FRONT_RIGHT_TURNING_MOTOR_PORT,
          DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_PORTS,
          DriveConstants.FRONT_RIGHT_TURNING_ENCODER_PORTS,
          DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
          DriveConstants.FRONT_RIGHT_TURNING_ENCODER_REVERSED);

  private final SwerveModule rearRight =
      new SwerveModule(
          DriveConstants.REAR_RIGHT_DRIVE_MOTOR_PORT,
          DriveConstants.REAR_RIGHT_TURNING_MOTOR_PORT,
          DriveConstants.REAR_RIGHT_DRIVE_ENCODER_PORTS,
          DriveConstants.REAR_RIGHT_TURNING_MOTOR_PORTS,
          DriveConstants.REAR_RIGHT_DRIVE_ENCODER_REVERSED,
          DriveConstants.REAR_RIGHT_TURNING_ENCODER_REVERSED);

  // The gyro sensor
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private final ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          gyro.getRotation2d(),
          getModulePositions());

  private Rotation2d rawGyroRotation = new Rotation2d();
  private final Field2d field = new Field2d();

  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Nothing to do
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        gyro.getRotation2d(),
        getModulePositions());

    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putData(field);

    SmartDashboard.putNumber("Gyro Angle", Math.IEEEremainder(gyro.getAngle(),360));
    SmartDashboard.putNumber("Pose X", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Y", odometry.getPoseMeters().getY());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        gyro.getRotation2d(),
        getModulePositions(),
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                DriveConstants.DRIVE_PERIOD));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot", rot);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.TRACK_WIDTH / 2.0, DriveConstants.WHEELBASE / 2.0),
      new Translation2d(DriveConstants.TRACK_WIDTH / 2.0, -DriveConstants.WHEELBASE / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH / 2.0, DriveConstants.WHEELBASE / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH / 2.0, -DriveConstants.WHEELBASE / 2.0)
    };
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()};
  }

  @Override
  public void simulationPeriodic() {
    // Simulate the swerve module drive and turn motors
    frontLeft.simulate();
    frontRight.simulate();
    rearLeft.simulate();
    rearRight.simulate();

    // Adapted from AdvantageKit SwerveDriveProject (FRC 6328 - Mechanical Advantage)
    // Update gyro angle using the angle delta from the kinematics and module deltas
    // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }
    Twist2d twist = kinematics.toTwist2d(moduleDeltas);
    rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    gyroSim.setAngle(-rawGyroRotation.getDegrees());
  }
}
