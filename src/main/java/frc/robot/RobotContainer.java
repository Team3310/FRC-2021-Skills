// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.auto.SlalomSkills;
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
    public final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // The driver's controller
    private final GameController m_driverController = new GameController(OIConstants.kDriverControllerPort, new Xbox());
    private SendableChooser<Command> autonTaskChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();
        configureAuton();
        configureDrive();
    }

    public void robotInit() {
        m_robotDrive.resetHeading();
        m_robotDrive.resetEncoders();
    }

    private void configureAuton() {
        autonTaskChooser = new SendableChooser<>();
        autonTaskChooser.addOption("Slalom Path", new SlalomSkills(m_robotDrive));
        SmartDashboard.putData("auton", autonTaskChooser);
    }

    private void configureDrive() {
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        RunCommand driveCommand = new RunCommand(
                () -> m_robotDrive.drive(-m_driverController.getLeftYAxis() * DriveConstants.kMaxSpeedMetersPerSecond,
                        -m_driverController.getLeftXAxis() * DriveConstants.kMaxSpeedMetersPerSecond,
                        -m_driverController.getRightXAxis() * 10.0, true));
        driveCommand.addRequirements(m_robotDrive);
        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(driveCommand);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
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

    public static Trajectory loadPathTrajectory(String JSONPath) {
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(JSONPath);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + JSONPath, ex.getStackTrace());
        }
        return trajectory;
    }

    public Command getAutonomousCommand() {
        return autonTaskChooser.getSelected();
    }
}
