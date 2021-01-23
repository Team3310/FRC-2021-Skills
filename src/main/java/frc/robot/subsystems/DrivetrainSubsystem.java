package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.common.control.HolonomicMotionProfiledTrajectoryFollower;
import frc.common.control.MaxAccelerationConstraint;
import frc.common.control.MaxVelocityConstraint;
import frc.common.control.PidConstants;
import frc.common.control.PidController;
import frc.common.drivers.Gyroscope;
import frc.common.drivers.SwerveModule;
import frc.common.math.RigidTransform2;
import frc.common.math.Vector2;
import frc.common.util.DrivetrainFeedforwardConstants;
import frc.common.util.HolonomicDriveSignal;
import frc.common.util.HolonomicFeedforward;
import frc.robot.drivers.Mk2SwerveModule;

public class DrivetrainSubsystem extends SwerveDrivetrain {
    private static final double TRACKWIDTH = 19.5;
    private static final double WHEELBASE = 23.5;

    private static final double MAX_VELOCITY = 12.0 * 12.0;

    private static final double FRONT_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-154.3);
    private static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-329.0);
    private static final double BACK_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-218.10);
    private static final double BACK_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-268.9);
    private static final double FRONT_LEFT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-170.2152486947372);
    private static final double FRONT_RIGHT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-43.55619048306742);
    private static final double BACK_LEFT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-237.47063008637048);
    private static final double BACK_RIGHT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-336.70093128378477);

    private static final PidConstants FOLLOWER_TRANSLATION_CONSTANTS = new PidConstants(0.05, 0.01, 0.0);
    private static final PidConstants FOLLOWER_ROTATION_CONSTANTS = new PidConstants(0.2, 0.01, 0.0);
    private static final HolonomicFeedforward FOLLOWER_FEEDFORWARD_CONSTANTS = new HolonomicFeedforward(
            new DrivetrainFeedforwardConstants(1.0 / (14.0 * 12.0), 0.0, 0.0)
    );

    private static final PidConstants SNAP_ROTATION_CONSTANTS = new PidConstants(0.3, 0.01, 0.0);

    private static final DrivetrainSubsystem instance = new DrivetrainSubsystem();

    private Mk2SwerveModule[] swerveModules;

    private HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            FOLLOWER_TRANSLATION_CONSTANTS,
            FOLLOWER_ROTATION_CONSTANTS,
            FOLLOWER_FEEDFORWARD_CONSTANTS
    );

    private PidController snapRotationController = new PidController(SNAP_ROTATION_CONSTANTS);
    private double snapRotation = Double.NaN;

    private double lastTimestamp = 0;

    private final Object lock = new Object();
    private HolonomicDriveSignal signal = new HolonomicDriveSignal(Vector2.ZERO, 0.0, false);
 
    private DrivetrainSubsystem() {
        double frontLeftAngleOffset = FRONT_LEFT_ANGLE_OFFSET_COMPETITION;
        double frontRightAngleOffset = FRONT_RIGHT_ANGLE_OFFSET_COMPETITION;
        double backLeftAngleOffset = BACK_LEFT_ANGLE_OFFSET_COMPETITION;
        double backRightAngleOffset = BACK_RIGHT_ANGLE_OFFSET_COMPETITION;
 
        Mk2SwerveModule frontLeftModule = new Mk2SwerveModule(
                new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                frontLeftAngleOffset,
                new Spark(0),
                new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(2)
        );
        frontLeftModule.setName("Front Left");

        Mk2SwerveModule frontRightModule = new Mk2SwerveModule(
                new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                frontRightAngleOffset,
                new Spark(3),
                new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(5)
        );
        frontRightModule.setName("Front Right");

        Mk2SwerveModule backLeftModule = new Mk2SwerveModule(
                new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                backLeftAngleOffset,
                new Spark(6),
                new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(8)
        );
        backLeftModule.setName("Back Left");

        Mk2SwerveModule backRightModule = new Mk2SwerveModule(
                new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                backRightAngleOffset,
                new Spark(9),
                new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(11)
        );
        backRightModule.setName("Back Right");

        swerveModules = new Mk2SwerveModule[]{
                frontLeftModule,
                frontRightModule,
                backLeftModule,
                backRightModule,
        };

        snapRotationController.setInputRange(0.0, 2.0 * Math.PI);
        snapRotationController.setContinuous(true);
        snapRotationController.setOutputRange(-0.5, 0.5);
    }

    public void setSnapRotation(double snapRotation) {
        synchronized (lock) {
            this.snapRotation = snapRotation;
        }
    }

    public void stopSnap() {
        synchronized (lock) {
            this.snapRotation = Double.NaN;
        }
    }

    @Override
    public void holonomicDrive(Vector2 translation, double rotation, boolean fieldOriented) {
        synchronized (lock) {
            this.signal = new HolonomicDriveSignal(translation, rotation, fieldOriented);
        }
    }

    @Override
    public synchronized void updateKinematics(double timestamp) {
        super.updateKinematics(timestamp);

        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;

        double localSnapRotation;
        synchronized (lock) {
            localSnapRotation = snapRotation;
        }
        RigidTransform2 currentPose = new RigidTransform2(
                getKinematicPosition(),
                getGyroscope().getAngle()
        );

        Optional<HolonomicDriveSignal> optSignal = follower.update(currentPose, getKinematicVelocity(),
                getGyroscope().getRate(), timestamp, dt);
        HolonomicDriveSignal localSignal;

        if (optSignal.isPresent()) {
            localSignal = optSignal.get();

        } else {
            synchronized (lock) {
                localSignal = this.signal;
            }
        }

        if (Math.abs(localSignal.getRotation()) < 0.1 && Double.isFinite(localSnapRotation)) {
            snapRotationController.setSetpoint(localSnapRotation);

            localSignal = new HolonomicDriveSignal(localSignal.getTranslation(),
                    snapRotationController.calculate(getGyroscope().getAngle().toRadians(), dt),
                    localSignal.isFieldOriented());
        } else {
            synchronized (lock) {
                snapRotation = Double.NaN;
            }
        }

        super.holonomicDrive(localSignal.getTranslation(), localSignal.getRotation(), localSignal.isFieldOriented());
    }

    @Override
    public void outputToSmartDashboard() {
        super.outputToSmartDashboard();
        for (Mk2SwerveModule module : swerveModules) {
            SmartDashboard.putNumber(String.format("%s Module Drive Current Draw", module.getName()), module.getDriveCurrent());
        }
    }

    public static DrivetrainSubsystem getInstance() {
        return instance;
    }

    @Override
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    @Override
    public Gyroscope getGyroscope() {
        return null; //Superstructure.getInstance().getGyroscope();
    }

    @Override
    public double getMaximumVelocity() {
        return 0;
    }

    @Override
    public double getMaximumAcceleration() {
        return 0;
    }

    @Override
    public void stop() {
        super.stop();
        synchronized (lock) {
            snapRotation = Double.NaN;
        }
    }

    public HolonomicMotionProfiledTrajectoryFollower getFollower() {
        return follower;
    }
}
