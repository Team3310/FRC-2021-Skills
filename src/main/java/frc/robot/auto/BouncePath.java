package frc.robot.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class BouncePath extends SequentialCommandGroup {

    public BouncePath(DriveSubsystem robotDrive){

        var thetaController =
                new ProfiledPIDController(
                        Constants.AutoConstants.kPThetaController, 0, 0.1, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addCommands(
                
                new SwerveControllerCommand(
                        RobotContainer.loadPathTrajectory("output/firstClimb.wpilib.json"),
                        robotDrive::getPose, // Functional interface to feed supplier
                        Constants.DriveConstants.kDriveKinematics,

                                // Position controllers
                        new PIDController(Constants.AutoConstants.kPXController, 0, 0.1),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0.1),
                        thetaController,
                        robotDrive::setModuleStates,
                        robotDrive),

                new SwerveControllerCommand(
                        RobotContainer.loadPathTrajectory("output/firstFall.wpilib.json"),
                        robotDrive::getPose, // Functional interface to feed supplier
                        Constants.DriveConstants.kDriveKinematics,

                        // Position controllers
                        new PIDController(Constants.AutoConstants.kPXController, 0, 0.1),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0.1),
                        thetaController,
                        robotDrive::setModuleStates,
                        robotDrive),

                new SwerveControllerCommand(
                        RobotContainer.loadPathTrajectory("output/secondClimb.wpilib.json"),
                        robotDrive::getPose, // Functional interface to feed supplier
                        Constants.DriveConstants.kDriveKinematics,

                        // Position controllers
                        new PIDController(Constants.AutoConstants.kPXController, 0, 0.1),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0.1),
                        thetaController,
                        robotDrive::setModuleStates,
                        robotDrive),

                new SwerveControllerCommand(
                        RobotContainer.loadPathTrajectory("output/secondFallAndLastClimb.wpilib.json"),
                        robotDrive::getPose, // Functional interface to feed supplier
                        Constants.DriveConstants.kDriveKinematics,
                        // Position controllers
                        new PIDController(Constants.AutoConstants.kPXController, 0, 0.1),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0.1),
                        thetaController,
                        robotDrive::setModuleStates,
                        robotDrive),

                new SwerveControllerCommand(
                        RobotContainer.loadPathTrajectory("output/finalFall.wpilib.json"),
                        robotDrive::getPose, // Functional interface to feed supplier
                        Constants.DriveConstants.kDriveKinematics,

                        // Position controllers
                        new PIDController(Constants.AutoConstants.kPXController, 0, 0.1),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0.1),
                        thetaController,
                        robotDrive::setModuleStates,
                        robotDrive),

                new SwerveControllerCommand(
                        RobotContainer.loadPathTrajectory("output/firstClimb.wpilib.json"),
                        robotDrive::getPose, // Functional interface to feed supplier
                        Constants.DriveConstants.kDriveKinematics,

                        // Position controllers
                        new PIDController(Constants.AutoConstants.kPXController, 0, 0.1),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0.1),
                        thetaController,
                        robotDrive::setModuleStates,
                        robotDrive),

                new StopTrajectory(robotDrive)
        );


    }
}
