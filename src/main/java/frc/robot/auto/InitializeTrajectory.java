package frc.robot.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class InitializeTrajectory extends CommandBase {
    private DriveSubsystem robotDrive;
    private Trajectory trajectory;

    public InitializeTrajectory(DriveSubsystem robotDrive, Trajectory trajectory) {
        this.robotDrive = robotDrive;
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        System.out.println("Initialize Trajectory");
        robotDrive.resetOdometry(trajectory.getInitialPose());
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
