package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class StopTrajectory extends CommandBase {
    DriveSubsystem robotDrive;

    StopTrajectory(DriveSubsystem robotDrive) {
        this.robotDrive = robotDrive;
    }

    @Override
    public void initialize() {
        System.out.println("Stopping Trajectory");
        robotDrive.setWheelSpeedPercent(0);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Trajectory Stopped");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
