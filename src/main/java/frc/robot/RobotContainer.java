// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.controller.GameController;
import frc.robot.controller.Xbox;
import frc.robot.subsystems.DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  private final GameController m_driverController = new GameController(OIConstants.kDriverControllerPort, new Xbox());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();


            // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
    RunCommand driveCommand = new RunCommand(
            () ->
                m_robotDrive.drive(
                    -m_driverController.getLeftYAxis()*9.0,
                    -m_driverController.getLeftXAxis()*9.0,
                    -m_driverController.getRightXAxis()*20.0,
                    true));
    driveCommand.addRequirements(m_robotDrive);
    // Configure default commands
    // Set the default drive command to split-stick arcade drive
   m_robotDrive.setDefaultCommand(driveCommand);
  }

  
  private double deadband(double input) {
    if (Math.abs(input) < 0.02) {
        return 0;
    } 
    return input;
}
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
      SmartDashboard.putData("Set Turn 0", new InstantCommand(() -> m_robotDrive.setTurnAngle(0)));
      SmartDashboard.putData("Set Turn 90", new InstantCommand(() -> m_robotDrive.setTurnAngle(90)));
      SmartDashboard.putData("Set Turn 180", new InstantCommand(() -> m_robotDrive.setTurnAngle(180)));
      SmartDashboard.putData("Set Turn 270", new InstantCommand(() -> m_robotDrive.setTurnAngle(270)));
      SmartDashboard.putData("Set Turn 360", new InstantCommand(() -> m_robotDrive.setTurnAngle(360)));
      SmartDashboard.putData("Set Wheel Speed 0", new InstantCommand(() -> m_robotDrive.setWheelSpeed(0)));
      SmartDashboard.putData("Set Wheel Speed 1", new InstantCommand(() -> m_robotDrive.setWheelSpeed(1)));
      SmartDashboard.putData("Set Wheel Speed 4", new InstantCommand(() -> m_robotDrive.setWheelSpeed(4)));
      SmartDashboard.putData("Set Wheel Speed 9", new InstantCommand(() -> m_robotDrive.setWheelSpeed(9)));
      SmartDashboard.putData("Set Wheel Speed 20", new InstantCommand(() -> m_robotDrive.setWheelSpeed(20)));
      SmartDashboard.putData("Reset Encoders", new InstantCommand(() -> m_robotDrive.resetEncoders()));
      SmartDashboard.putData("Reset Gyro", new InstantCommand(() -> m_robotDrive.resetHeading()));
      SmartDashboard.putData("Enable Drive", new InstantCommand(() -> m_robotDrive.setDriveEnabled(true)));
      SmartDashboard.putData("Disable Drive", new InstantCommand(() -> m_robotDrive.setDriveEnabled(false)));
      SmartDashboard.putData("Set percent 100", new InstantCommand(() -> m_robotDrive.setWheelSpeedPercent(100)));
      SmartDashboard.putData("Set percent 0", new InstantCommand(() -> m_robotDrive.setWheelSpeedPercent(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
