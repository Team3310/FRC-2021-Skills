package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.common.drivers.Gyroscope;
import frc.common.math.Vector2;

public abstract class Drivetrain extends Subsystem {
	public abstract Gyroscope getGyroscope();
	
	public abstract double getMaximumVelocity();
	public abstract double getMaximumAcceleration();

	@Override
	public abstract void updateKinematics(double timestamp);

	public abstract Vector2 getKinematicPosition();

	public abstract Vector2 getKinematicVelocity();

	public void outputToSmartDashboard() {
		SmartDashboard.putString("Drivetrain position", getKinematicPosition().toString());
		SmartDashboard.putNumber("Drivetrain X velocity", getKinematicVelocity().x);
		SmartDashboard.putNumber("Drivetrain Y velocity", getKinematicVelocity().y);

		SmartDashboard.putNumber("Drivetrain angle", getGyroscope().getAngle().toDegrees());
	}
	
	public void zeroSensors() {}
}