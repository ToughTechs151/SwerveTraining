// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  private final Spark driveMotor;
  private final Spark turningMotor;

  private final Encoder driveEncoder;
  private final Encoder turningEncoder;

  private final EncoderSim driveEncoderSim;
  private final EncoderSim turningEncoderSim;

  private final String moduleName;

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private final PIDController drivePIDController =
      new PIDController(ModuleConstants.KP_MODULE_DRIVE_CONTROLLER, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.KP_MODULE_TURNING_CONTROLLER,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.MAX_MODULE_ANGULAR_SPEED_RADS_PER_SEC,
              ModuleConstants.MAX_MODULE_ANGULAR_ACCEL_RADS_PER_SEC2));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward driveFeedforwardController = new SimpleMotorFeedforward(
      ModuleConstants.KS_DRIVE_VOLTS, 
      ModuleConstants.KV_DRIVE_VOLTS_SECONDS_PER_METER);

  private DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
  private DCMotorSim turnSim = new DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);
  private static final double LOOP_PERIOD_SECS = 0.02;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int[] driveEncoderChannels,
      int[] turningEncoderChannels,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    driveMotor = new Spark(driveMotorChannel);
    turningMotor = new Spark(turningMotorChannel);
    moduleName = "Swerve" + Integer.toString(driveMotorChannel);

    driveEncoder = new Encoder(driveEncoderChannels[0], driveEncoderChannels[1]);
    turningEncoder = new Encoder(turningEncoderChannels[0], turningEncoderChannels[1]);

    driveEncoderSim = new EncoderSim(driveEncoder);
    turningEncoderSim = new EncoderSim(turningEncoder);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    driveEncoder.setDistancePerPulse(ModuleConstants.DRIVE_ENCODER_DISTANCE_PER_PULSE);

    // Set whether drive encoder should be reversed or not
    driveEncoder.setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    turningEncoder.setDistancePerPulse(ModuleConstants.TURNING_ENCODER_DISTANCE_PER_PULSE);

    // Set whether turning encoder should be reversed or not
    turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getRate(), new Rotation2d(turningEncoder.getDistance()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getDistance(), new Rotation2d(turningEncoder.getDistance()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(turningEncoder.getDistance());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        drivePIDController.calculate(driveEncoder.getRate(), state.speedMetersPerSecond);

    final double driveFeedforward = driveFeedforwardController.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        turningPIDController.calculate(turningEncoder.getDistance(), state.angle.getRadians());

    driveAppliedVolts = driveOutput + driveFeedforward;
    turnAppliedVolts = turnOutput;
    driveMotor.setVoltage(driveAppliedVolts);
    turningMotor.setVoltage(turnAppliedVolts);
 
    SmartDashboard.putNumber(moduleName + "/State Speed", state.speedMetersPerSecond);
    SmartDashboard.putNumber(moduleName + "/State Angle", state.angle.getDegrees());
    SmartDashboard.putNumber(moduleName + "/Turn Rotation", Math.IEEEremainder(encoderRotation.getDegrees(),360));
    SmartDashboard.putNumber(moduleName + "/Drive Rate", driveEncoder.getRate());
    SmartDashboard.putNumber(moduleName + "/Drive Distance", driveEncoder.getDistance());
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.reset();
    turningEncoder.reset();
  }

  public void simulate(){
    driveSim.setInputVoltage(driveAppliedVolts);
    turnSim.setInputVoltage(turnAppliedVolts);

    driveSim.update(LOOP_PERIOD_SECS);
    turnSim.update(LOOP_PERIOD_SECS);

    driveEncoderSim.setDistance(driveSim.getAngularPositionRad() * ModuleConstants.WHEEL_DIAMETER_METERS/2);
    driveEncoderSim.setRate(driveSim.getAngularVelocityRadPerSec() * ModuleConstants.WHEEL_DIAMETER_METERS/2);
    turningEncoderSim.setDistance(turnSim.getAngularPositionRad());
    turningEncoderSim.setRate(turnSim.getAngularVelocityRadPerSec());
  }
}
