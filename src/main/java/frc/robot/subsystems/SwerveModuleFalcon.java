// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseMotorControllerConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

public class SwerveModuleFalcon {

  private final TalonFX m_driveMotor;
  private final TalonFX m_turnMotor;
  private final CANCoder canEncoder;


  private double targetAngle;

  private static final int kTurnMotionMagicSlot = 0;
  private static final int kWheelVelocitySlot = 0;
  private static final double TWO_PI = 2*Math.PI;
  private static final double TURN_ZERO_MULTIPLIER = 1000000.0;
  private double zeroTurnOffsetTicks = 0;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModuleFalcon(int driveMotorChannel, int turningMotorChannel, boolean isRight, int canCoderID) {

    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turnMotor = new TalonFX(turningMotorChannel);
    canEncoder = new CANCoder(canCoderID);

    TalonFXConfiguration configsTurn = new TalonFXConfiguration();
    TalonFXConfiguration configsDrive = new TalonFXConfiguration();

    configsTurn.remoteFilter1.remoteSensorDeviceID = canCoderID;
    configsTurn.remoteFilter1.remoteSensorSource = RemoteSensorSource.CANCoder;
    configsTurn.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();

    configsDrive.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    m_driveMotor.configAllSettings(configsDrive);
    m_turnMotor.configAllSettings(configsTurn);

    m_driveMotor.setInverted(isRight ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise);
    m_turnMotor.setInverted(TalonFXInvertType.CounterClockwise);

    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turnMotor.setNeutralMode(NeutralMode.Brake);

    m_driveMotor.enableVoltageCompensation(true);
    m_driveMotor.configVoltageCompSaturation(12.0);
    m_driveMotor.configPeakOutputForward(+1.0f);
    m_driveMotor.configPeakOutputReverse(-1.0f);

    m_turnMotor.enableVoltageCompensation(true);
    m_turnMotor.configVoltageCompSaturation(12.0);
    m_turnMotor.configPeakOutputForward(+1.0f);
    m_turnMotor.configPeakOutputReverse(-1.0f);

    SupplyCurrentLimitConfiguration supplyCurrentConfigs = new SupplyCurrentLimitConfiguration();
    supplyCurrentConfigs.currentLimit = 60;
    supplyCurrentConfigs.enable = false;

    m_driveMotor.configSupplyCurrentLimit(supplyCurrentConfigs);
    m_turnMotor.configSupplyCurrentLimit(supplyCurrentConfigs);

    StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
    statorCurrentConfigs.currentLimit = 60;
    statorCurrentConfigs.triggerThresholdCurrent = 80;
    statorCurrentConfigs.triggerThresholdTime = 0.5;
    statorCurrentConfigs.enable = true;

    m_driveMotor.configStatorCurrentLimit(statorCurrentConfigs);
    m_turnMotor.configStatorCurrentLimit(statorCurrentConfigs);

    m_driveMotor.config_kF(kWheelVelocitySlot, 0.04);
    m_driveMotor.config_kP(kWheelVelocitySlot, 0.03);
    m_driveMotor.config_kI(kWheelVelocitySlot, 0.00005);
    m_driveMotor.config_kD(kWheelVelocitySlot, 1.5);
  
    m_driveMotor.selectProfileSlot(kWheelVelocitySlot, 0);

    m_turnMotor.configMotionCruiseVelocity(12000);
    m_turnMotor.configMotionAcceleration(28000);
    m_turnMotor.configMotionSCurveStrength(4);

    m_turnMotor.config_kF(kTurnMotionMagicSlot, 0.1);
    m_turnMotor.config_kP(kTurnMotionMagicSlot, 3.0);
    m_turnMotor.config_kI(kTurnMotionMagicSlot, 0.003);
    m_turnMotor.config_kD(kTurnMotionMagicSlot, 2.0);
    m_turnMotor.config_IntegralZone(kTurnMotionMagicSlot, (int)(5.0 * turnDegreesToTicks(5)));

    m_turnMotor.selectProfileSlot(kTurnMotionMagicSlot, 0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMetersPerSecond(), new Rotation2d(getTurnWheelAngleRadians()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
//   SwerveModuleState state =
//            SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getTurnWheelAngleDegrees()));
    double targetAngleDegrees = desiredState.angle.getDegrees();
    double currentAngleDegrees = getTurnWheelAngleDegrees();
    double targetSpeed = desiredState.speedMetersPerSecond;
    double error = Math.IEEEremainder(targetAngleDegrees-currentAngleDegrees, 360.0);

    if(Math.abs(error) > 90.0) {
      error -= Math.copySign(180, error);
      targetSpeed = -targetSpeed;
    }

    // Calculate the turning motor output from the turning PID controller.
        m_driveMotor.set(TalonFXControlMode.Velocity, driveMetersPerSecondToTicksPer100ms(targetSpeed));
        m_turnMotor.set(TalonFXControlMode.MotionMagic, turnDegreesToTicks(currentAngleDegrees + error),
        DemandType.ArbitraryFeedForward, 0.0);
  }

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. If this is used with the PIDController class's
   * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }

  public void setControlOff() {
    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(TalonFXControlMode.PercentOutput,0);
    m_turnMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void setTurnAngle(double angleDegrees) {
    targetAngle = getClosestTargetAngle(angleDegrees,getTurnWheelAngleDegrees());
    m_turnMotor.set(TalonFXControlMode.MotionMagic, turnDegreesToTicks(targetAngle) - zeroTurnOffsetTicks, DemandType.ArbitraryFeedForward, 0.0);
  }

  public void setWheelSpeed(double metersPerSec) {
    m_driveMotor.set(TalonFXControlMode.Velocity, driveMetersPerSecondToTicksPer100ms(metersPerSec));
  }
  public void setWheelSpeedPercentBus(double percent) {
    m_driveMotor.set(TalonFXControlMode.PercentOutput, percent);
  }

  // Zeros all the SwerveModule encoders.
  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0);
    updateTurnZeroOffset();
  }

  public void updateTurnZeroOffset() {
    double referenceZeroTurnDegrees = (double)(m_turnMotor.configGetCustomParam(0)) / TURN_ZERO_MULTIPLIER;
    double currentTurnDegrees = canEncoder.getAbsolutePosition();
    zeroTurnOffsetTicks = turnDegreesToTicks(currentTurnDegrees - referenceZeroTurnDegrees);
  }

  public void saveTurnZero() {
    m_turnMotor.configSetCustomParam((int)(canEncoder.getAbsolutePosition() * TURN_ZERO_MULTIPLIER), 0);
  }

  // Returns drive sensor velocity in ticks per 100ms
  public double getDriveVelocityNativeUnits() {
    return m_driveMotor.getSelectedSensorVelocity(0);
  }

  // Returns turn sensor velocity in ticks per 100ms
  public double getTurnVelocityNativeUnits() {
    return m_turnMotor.getSelectedSensorVelocity(0);
  }

  // Returns drive sensor position in ticks
  public double getDrivePositionNativeUnits() {
    return m_driveMotor.getSelectedSensorPosition(0);
  }

  // Returns turn sensor position in ticks
  public double getTurnPositionNativeUnits() {
    return m_turnMotor.getSelectedSensorPosition(0) - zeroTurnOffsetTicks;
  }// changed from 0

  // Takes that times the wheel has rotated * by the circumference of the wheel to
  // get its distance traveled in inches
  public static double driveWheelRotationsToInches(double rotations) {
    return rotations * (Constants.DriveConstants.kWheelDiameterInches * Math.PI);
  }

  public static double driveWheelInchesToRotations(double inches) {
    return inches / (Constants.DriveConstants.kWheelDiameterInches * Math.PI);
  }

  public static double driveWheelInchesToTicks(double inches) {
    return driveWheelRotationsToTicks(driveWheelInchesToRotations(inches));
  }

  public static double driveWheelRotationsToTicks(double wheelRotations) {
    return driveEncoderRotationsToTicks(wheelRotations * Constants.DriveConstants.DRIVE_OUTPUT_TO_ENCODER_RATIO);
  }

  // Takes inches and converts it to meters using units class
  public static double inchesToMeters(double inches) {
    return Units.inchesToMeters(inches);
  }

  // Takes meters and converts it to inches using units class
  public static double metersToInches(double meters) {
    return Units.metersToInches(meters);
  }

  // Takes the wheel angle in degrees and converts to native units of ticks
  public static double turnDegreesToTicks(double angleDegrees) {
    return turnWheelRotationsToTicks(angleDegrees / 360.0);
  }

  public static double turnWheelRotationsToTicks(double wheelRotations) {
    return turnEncoderRotationsToTicks(wheelRotations * Constants.DriveConstants.TURN_OUTPUT_TO_ENCODER_RATIO);
  }

  public static double driveEncoderRotationsToTicks(double encoderRotations) {
    return encoderRotations * Constants.DriveConstants.DRIVE_ENCODER_TICKS_PER_MOTOR_REVOLUTION;
  }
  public static double turnEncoderRotationsToTicks(double encoderRotations) {
    return encoderRotations * Constants.DriveConstants.TURN_ENCODER_TICKS_PER_MOTOR_REVOLUTION;
  }


  // Takes the sensor velocity of an encoder * by 10 to get ticks per second / the
  // encoder PPR to get encoder rotations
  // per second and then uses the rotations to inches functions to get inches per
  // second
  private static double driveWheelTicksPer100msToInchesPerSec(double ticks_100ms) {
    return driveWheelRotationsToInches(
        ticks_100ms * 10.0 / Constants.DriveConstants.DRIVE_ENCODER_TICKS_PER_MOTOR_REVOLUTION);
  }

  private static double driveWheelInchesPerSecToTicksPer100ms(double inchesPerSec) {
    return driveWheelInchesToTicks(inchesPerSec) / 10.0;
  }

  // Returns drive inches per second using the sensor velocity and the
  // ticksToInches conversion method
  public double getDriveInchesPerSecond() {
    return driveWheelTicksPer100msToInchesPerSec(getDriveVelocityNativeUnits())
        / Constants.DriveConstants.DRIVE_OUTPUT_TO_ENCODER_RATIO;
  }

  // Returns turn inches per second using the sensor velocity and the
  // ticksToInches conversion method
  public double getTurnInchesPerSecond() {
    return driveWheelTicksPer100msToInchesPerSec(getTurnVelocityNativeUnits())
        / Constants.DriveConstants.TURN_OUTPUT_TO_ENCODER_RATIO;
  }

  // Returns drive meters per second using inchesPerSecond calculation and
  // inchesToMeters method
  public double getDriveMetersPerSecond() {
    return inchesToMeters(getDriveInchesPerSecond());
  }

  public double driveMetersPerSecondToTicksPer100ms(double metersPerSec) {
    return driveWheelInchesPerSecToTicksPer100ms(metersToInches(metersPerSec));
  }

  // Returns turn meters per second using inchesPerSecond calculation and
  // inchesToMeters method
  public double getTurnMetersPerSecond() {
    return inchesToMeters(getTurnInchesPerSecond());
  }

  // Sensors positions in ticks / Pulses per Revolution of the Encoder = Encoder
  // Rotations (If ratio is 1:1)
  public double getDriveEncoderRotations() {
    return getDrivePositionNativeUnits() / Constants.DriveConstants.DRIVE_ENCODER_TICKS_PER_MOTOR_REVOLUTION;
  }

  public double getTurnEncoderRotations() {
    return getTurnPositionNativeUnits() / Constants.DriveConstants.TURN_ENCODER_TICKS_PER_MOTOR_REVOLUTION;
  }

  // Wheel Rotations = Encoder Rotations (If ratio is 1:1)
  public double getDriveWheelRotations() {
    return getDriveEncoderRotations() / Constants.DriveConstants.DRIVE_OUTPUT_TO_ENCODER_RATIO;
  }

  public double getTurnWheelRotations() {
    return getTurnEncoderRotations() / Constants.DriveConstants.TURN_OUTPUT_TO_ENCODER_RATIO;
  }

  public double getTurnWheelAngleDegrees() {
    return getTurnWheelRotations() * 360.0;
  }

  public double getTurnWheelAngleRadians() {
    return getTurnWheelRotations() * 2.0 * Math.PI;
  }

  // Returns drive distance traveled in inches by taking wheel rotations and
  // converting it to inches
  public double getDriveWheelDistanceInches() {
    return driveWheelRotationsToInches(getDriveWheelRotations());
  }

  // Returns turn distance traveled in inches by taking wheel rotations and
  // converting it to inches
  public double getTurnWheelDistanceInches() {
    return driveWheelRotationsToInches(getTurnWheelRotations());
  }

  // Returns drive distance traveled in meters using calculated inches distances
  // and inchesToMeters conversion
  public double getDriveWheelDistanceMeters() {
    return inchesToMeters(getDriveWheelDistanceInches());
  }

  // Returns turn distance traveled in meters using calculated inches distances
  // and inchesToMeters conversion
  public double getTurnWheelDistanceMeters() {
    return inchesToMeters(getTurnWheelDistanceInches());
  }

  double normalize(double radians) {
    double normalized = radians % TWO_PI;
    normalized = (normalized+TWO_PI) % TWO_PI;
    return normalized <= Math.PI ? normalized : normalized - TWO_PI;
  }

  double getClosestTargetAngle(double targetAngleDegrees, double currentAngleDegrees) {
    double error = Math.IEEEremainder(targetAngleDegrees-currentAngleDegrees, 360);

    return currentAngleDegrees + error;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public double getAbsoluteCanPosition() {
    return canEncoder.getAbsolutePosition();
  }

  public double getCanPosition() {
    return canEncoder.getPosition();
  }

}
