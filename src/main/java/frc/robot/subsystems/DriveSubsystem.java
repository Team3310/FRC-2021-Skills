// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModuleFalcon m_frontLeft =
      new SwerveModuleFalcon(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
              true);

  private final SwerveModuleFalcon m_frontRight =
      new SwerveModuleFalcon(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
              false);

  private final SwerveModuleFalcon m_rearLeft =
      new SwerveModuleFalcon(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
              true);
    
  private final SwerveModuleFalcon m_rearRight =
      new SwerveModuleFalcon(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
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

  // The gyro sensor
  private PigeonIMU m_gyro = new PigeonIMU(DriveConstants.kGyroPort);
  private double[] xyz_dps = new double[3];

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(m_gyro.getFusedHeading()));

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_gyro.configFactoryDefault();
    resetHeading();
    resetEncoders();
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
    SmartDashboard.putNumber("Front Left Angle deg", m_frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("Rear Right Angle deg", m_rearRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("Rear Left Angle deg", m_rearLeft.getState().angle.getDegrees());
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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rot = rot;
    if (isDriveEnabled) {
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getHeading()))
              : new ChassisSpeeds(xSpeed, ySpeed, rot));
      SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
      speedCommandLF = swerveModuleStates[0].speedMetersPerSecond;
      rotCommandLF = swerveModuleStates[0].angle.getDegrees();
      speedCommandLB = swerveModuleStates[2].speedMetersPerSecond;
      rotCommandLB = swerveModuleStates[2].angle.getDegrees();
      speedCommandRF = swerveModuleStates[1].speedMetersPerSecond;
      rotCommandRF = swerveModuleStates[1].angle.getDegrees();
      speedCommandRB = swerveModuleStates[3].speedMetersPerSecond;
      rotCommandRB = swerveModuleStates[3].angle.getDegrees();

      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_rearLeft.setDesiredState(swerveModuleStates[2]);
      m_rearRight.setDesiredState(swerveModuleStates[3]);
    } 
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
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    //SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
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
