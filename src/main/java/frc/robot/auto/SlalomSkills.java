package frc.robot.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class SlalomSkills extends SequentialCommandGroup {

    public SlalomSkills(DriveSubsystem driveSubsystem) {
        var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory trajectory = RobotContainer.loadPathTrajectory("output/Slalom1.wpilib.json");

        addCommands(
                new InstantCommand(() -> driveSubsystem.resetOdometry(trajectory.getInitialPose())),
                new SwerveControllerCommand(trajectory, driveSubsystem::getPose, // Functional interface to feed
                                                                                 // supplier
                        DriveConstants.kDriveKinematics,

                        // Position controllers
                        new PIDController(AutoConstants.kPXController, 0, 0),
                        new PIDController(AutoConstants.kPYController, 0, 0), thetaController,
                        driveSubsystem::setModuleStates, driveSubsystem),
                new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, false)));
    }
}
