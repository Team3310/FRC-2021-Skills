// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModuleFalcon.DriveControlMode;
import frc.robot.utilities.SwerveDriveOdometryBHR;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModuleFalcon m_frontLeft =
      new SwerveModuleFalcon(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftCanCoderPort, 
          DriveConstants.absoluteTurnZeroDegLeftFront, 
          true);

  private final SwerveModuleFalcon m_frontRight =
      new SwerveModuleFalcon(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightCanCoderPort, 
          DriveConstants.absoluteTurnZeroDegRightFront, 
          false);

  private final SwerveModuleFalcon m_rearLeft =
      new SwerveModuleFalcon(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftCanCoderPort, 
          DriveConstants.absoluteTurnZeroDegLeftRear, 
          true);
    
  private final SwerveModuleFalcon m_rearRight =
      new SwerveModuleFalcon(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightCanCoderPort, 
          DriveConstants.absoluteTurnZeroDegRightRear, 
          false);

  private boolean isDriveEnabled = false;
  private double xSpeed;
  private double ySpeed;
  private double rot;
  private double speedCommandLF;
  private double speedCommandRF;
  private double speedCommandLB;
  private double speedCommandRB;
  private double rotCommandLF;
  private double rotCommandRF;
  private double rotCommandLB;
  private double rotCommandRB;
  private Translation2d centerPivot = new Translation2d();
  private Translation2d rightFrontPivot = new Translation2d(Constants.DriveConstants.kTrackWidth/2.0, -Constants.DriveConstants.kTrackWidth/2.0);
  private Translation2d leftFrontPivot = new Translation2d(Constants.DriveConstants.kTrackWidth/2.0, Constants.DriveConstants.kTrackWidth/2.0);

  // The gyro sensor
  private PigeonIMU m_gyro = new PigeonIMU(DriveConstants.kGyroPort);
  private double[] xyz_dps = new double[3];

  // Odometry class for tracking robot pose
  SwerveDriveOdometryBHR m_odometry =
      new SwerveDriveOdometryBHR(DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(m_gyro.getFusedHeading()));

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_gyro.configFactoryDefault();
  }

  @Override
  public void periodic() {
      // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    );
      
    SmartDashboard.putNumber("Heading deg", getHeading());
    SmartDashboard.putNumber("Heading Rate deg per s", getTurnRate());
    SmartDashboard.putNumber("Front Right Angle deg", m_frontRight.getTurnWheelAngleDegrees());
    SmartDashboard.putNumber("Front Left Angle deg", m_frontLeft.getTurnWheelAngleDegrees());
    SmartDashboard.putNumber("Rear Right Angle deg", m_rearRight.getTurnWheelAngleDegrees());
    SmartDashboard.putNumber("Rear Left Angle deg", m_rearLeft.getTurnWheelAngleDegrees());
    SmartDashboard.putNumber("front right native units", m_frontRight.getTurnPositionNativeUnits());
    SmartDashboard.putNumber("Front Right Speed m per s", m_rearRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot", rot);
    SmartDashboard.putNumber("Left Front commanded Speed", speedCommandLF);
    SmartDashboard.putNumber("Left Front commanded Angle", rotCommandLF);
    SmartDashboard.putNumber("Left back commanded Speed", speedCommandLB);
    SmartDashboard.putNumber("Left back commanded Angle", rotCommandLB);
    SmartDashboard.putNumber("Right Front commanded Speed", speedCommandRF);
    SmartDashboard.putNumber("Right Front commanded Angle", rotCommandRF);
    SmartDashboard.putNumber("Right Back commanded Speed", speedCommandRB);
    SmartDashboard.putNumber("Right Back commanded Angle", rotCommandRB);
    SmartDashboard.putNumber("Front right target angle", m_frontRight.getTargetAngle());
    SmartDashboard.putNumber("Front right travel distance inches", m_frontRight.getDriveWheelDistanceInches());
    SmartDashboard.putNumber("Front right travel distance meters", m_frontRight.getDriveWheelDistanceMeters());
    SmartDashboard.putNumber("Front Right Speed inches per s", m_frontRight.getDriveInchesPerSecond());
    SmartDashboard.putNumber("Front Right Speed Meters per s", m_frontRight.getDriveMetersPerSecond());
    SmartDashboard.putNumber("Front Right get Absolute can encoder pose", m_frontRight.getAbsoluteCanPosition());
    SmartDashboard.putNumber("Front Right get can encoder pose", m_frontRight.getCanPosition());
    SmartDashboard.putNumber("Front Right get offset", m_frontRight.getOffsetDeg());
    SmartDashboard.putNumber("Front Left get Absolute can encoder pose", m_frontLeft.getAbsoluteCanPosition());
    SmartDashboard.putNumber("Front Left get can encoder pose", m_frontLeft.getCanPosition());
    SmartDashboard.putNumber("Front Left get offset", m_frontLeft.getOffsetDeg());
    SmartDashboard.putNumber("Back Right get Absolute can encoder pose", m_rearRight.getAbsoluteCanPosition());
    SmartDashboard.putNumber("Back Right get can encoder pose", m_rearRight.getCanPosition());
    SmartDashboard.putNumber("Back Right get offset", m_rearRight.getOffsetDeg());
    SmartDashboard.putNumber("Back Left get Absolute can encoder pose", m_rearLeft.getAbsoluteCanPosition());
    SmartDashboard.putNumber("Back Left get can encoder pose", m_rearLeft.getCanPosition());
    SmartDashboard.putNumber("Back Left get offset", m_rearLeft.getOffsetDeg());
  }

  public void setTurnAngle(double angleDegrees) {
    m_frontRight.setTurnAngle(angleDegrees);
    m_frontLeft.setTurnAngle(angleDegrees);
    m_rearRight.setTurnAngle(angleDegrees);
    m_rearLeft.setTurnAngle(angleDegrees);
  }

  public void setWheelSpeed(double metersPerSec) {
    m_frontRight.setWheelSpeed(metersPerSec);
    m_frontLeft.setWheelSpeed(metersPerSec);
    m_rearRight.setWheelSpeed(metersPerSec);
    m_rearLeft.setWheelSpeed(metersPerSec);
  }
  public void setWheelSpeedPercent(double percent) {
    m_frontRight.setWheelSpeed(percent);
    m_frontLeft.setWheelSpeed(percent);
    m_rearRight.setWheelSpeed(percent);
    m_rearLeft.setWheelSpeed(percent);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean leftPivot, boolean rightPivot) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rot = rot;

    Translation2d pivot = centerPivot;
    if (leftPivot) {
      pivot = leftFrontPivot;
    } else if (rightPivot) {
      pivot = rightFrontPivot;
    }

    if (isDriveEnabled) {
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getHeading()))
              : new ChassisSpeeds(xSpeed, ySpeed, rot), pivot);
      SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
      speedCommandLF = swerveModuleStates[0].speedMetersPerSecond;
      rotCommandLF = swerveModuleStates[0].angle.getDegrees();
      speedCommandLB = swerveModuleStates[2].speedMetersPerSecond;
      rotCommandLB = swerveModuleStates[2].angle.getDegrees();
      speedCommandRF = swerveModuleStates[1].speedMetersPerSecond;
      rotCommandRF = swerveModuleStates[1].angle.getDegrees();
      speedCommandRB = swerveModuleStates[3].speedMetersPerSecond;
      rotCommandRB = swerveModuleStates[3].angle.getDegrees();

      DriveControlMode controlMode = DriveControlMode.Voltage;
      m_frontLeft.setDesiredState(swerveModuleStates[0], controlMode);
      m_frontRight.setDesiredState(swerveModuleStates[1], controlMode);
      m_rearLeft.setDesiredState(swerveModuleStates[2], controlMode);
      m_rearRight.setDesiredState(swerveModuleStates[3], controlMode);
    } 
  }

  public static Translation2d getFieldRelativePivot(Translation2d robotRelativePivot, Rotation2d robotAngle) {
    return robotRelativePivot.rotateBy(robotAngle);
  }

  public void setDriveEnabled(boolean isEnabled) {
    isDriveEnabled = isEnabled;
    if (!isEnabled) {
      m_frontLeft.setControlOff();
      m_frontRight.setControlOff();
      m_rearLeft.setControlOff();
      m_rearRight.setControlOff();
    }
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, DriveControlMode controlMode) {
    SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0], controlMode);
    m_frontRight.setDesiredState(desiredStates[1], controlMode);
    m_rearLeft.setDesiredState(desiredStates[2], controlMode);
    m_rearRight.setDesiredState(desiredStates[3], controlMode);
  }

  public void setModuleStatesVelocityDriveControl(SwerveModuleState[] desiredStates) {
    setModuleStates(desiredStates, DriveControlMode.Velocity);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }


  /** Zeroes the heading of the robot. */
  public void resetHeading() {
    m_gyro.setYaw(0);
    m_gyro.setFusedHeading(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getFusedHeading();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    m_gyro.getRawGyro(xyz_dps);
    return xyz_dps[0];
  }
}
