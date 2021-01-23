package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Subsystem extends SubsystemBase {

	public void writeToLog() {}

	public void updateKinematics(double timestamp) {}

	public void resetKinematics(double timestamp) {}

	public abstract void outputToSmartDashboard();

	public abstract void stop();

	public abstract void zeroSensors();
}
