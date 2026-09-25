package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIONavX;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleIOSparkMax;
import java.util.Arrays;
import java.util.Objects;

/** Four-module swerve kinematics, wheel control, heading, and pose estimation. */
public final class SwerveDriveSubsystem extends SubsystemBase implements AutoCloseable {
    private static final int MODULE_COUNT = 4;

    private final GyroIO gyro;
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field = new Field2d();

    public SwerveDriveSubsystem(
            GyroIO gyro,
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight) {
        this.gyro = Objects.requireNonNull(gyro);
        modules = new SwerveModule[] {
            Objects.requireNonNull(frontLeft),
            Objects.requireNonNull(frontRight),
            Objects.requireNonNull(backLeft),
            Objects.requireNonNull(backRight)
        };
        kinematics = new SwerveDriveKinematics(
            SwerveConstants.FRONT_LEFT_LOCATION,
            SwerveConstants.FRONT_RIGHT_LOCATION,
            SwerveConstants.BACK_LEFT_LOCATION,
            SwerveConstants.BACK_RIGHT_LOCATION);

        updateModuleInputs();
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            gyro.getHeading(),
            getModulePositions(),
            Pose2d.kZero);
        SmartDashboard.putData("Swerve/Field", field);
    }

    public static SwerveDriveSubsystem createReal() {
        return new SwerveDriveSubsystem(
            new GyroIONavX(),
            new SwerveModule(new SwerveModuleIOSparkMax(SwerveConstants.MODULE_CONFIGS.get(0))),
            new SwerveModule(new SwerveModuleIOSparkMax(SwerveConstants.MODULE_CONFIGS.get(1))),
            new SwerveModule(new SwerveModuleIOSparkMax(SwerveConstants.MODULE_CONFIGS.get(2))),
            new SwerveModule(new SwerveModuleIOSparkMax(SwerveConstants.MODULE_CONFIGS.get(3))));
    }

    public void drive(
            double xSpeedMetersPerSecond,
            double ySpeedMetersPerSecond,
            double angularSpeedRadiansPerSecond,
            boolean fieldRelative) {
        ChassisSpeeds speeds;
        if (fieldRelative && gyro.isConnected() && !gyro.isCalibrating()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedMetersPerSecond,
                ySpeedMetersPerSecond,
                angularSpeedRadiansPerSecond,
                gyro.getHeading());
        } else {
            speeds = new ChassisSpeeds(
                xSpeedMetersPerSecond,
                ySpeedMetersPerSecond,
                angularSpeedRadiansPerSecond);
        }

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(
            speeds,
            SwerveConstants.LOOP_PERIOD_SECONDS);
        setModuleStates(kinematics.toSwerveModuleStates(discreteSpeeds));
    }

    public void setModuleStates(SwerveModuleState[] requestedStates) {
        if (requestedStates == null || requestedStates.length != MODULE_COUNT) {
            throw new IllegalArgumentException("Swerve requires exactly four module states");
        }

        SwerveModuleState[] states = Arrays.stream(requestedStates)
            .map(state -> {
                Objects.requireNonNull(state, "Module state cannot be null");
                return new SwerveModuleState(state.speedMetersPerSecond, state.angle);
            })
            .toArray(SwerveModuleState[]::new);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED_MPS);

        for (int index = 0; index < MODULE_COUNT; index++) {
            modules[index].setDesiredState(states[index]);
        }
    }

    public SwerveModuleState[] getDesiredModuleStates() {
        return Arrays.stream(modules)
            .map(SwerveModule::getLastDesiredState)
            .toArray(SwerveModuleState[]::new);
    }

    public SwerveModuleState[] getMeasuredModuleStates() {
        return Arrays.stream(modules)
            .map(SwerveModule::getState)
            .toArray(SwerveModuleState[]::new);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(gyro.getHeading(), getModulePositions(), pose);
        field.setRobotPose(pose);
    }

    public void zeroHeading() {
        gyro.zeroYaw();
    }

    public Rotation2d getHeading() {
        return gyro.getHeading();
    }

    public boolean isGyroConnected() {
        return gyro.isConnected();
    }

    public boolean isGyroCalibrating() {
        return gyro.isCalibrating();
    }

    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    @Override
    public void periodic() {
        updateModuleInputs();
        Pose2d estimatedPose = poseEstimator.update(gyro.getHeading(), getModulePositions());
        field.setRobotPose(estimatedPose);
        publishTelemetry();
    }

    private void updateModuleInputs() {
        for (SwerveModule module : modules) {
            module.updateInputs();
        }
    }

    private SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules)
            .map(SwerveModule::getPosition)
            .toArray(SwerveModulePosition[]::new);
    }

    private void publishTelemetry() {
        SmartDashboard.putBoolean("Swerve/GyroConnected", gyro.isConnected());
        SmartDashboard.putBoolean("Swerve/GyroCalibrating", gyro.isCalibrating());
        SmartDashboard.putNumber("Swerve/HeadingDegrees", gyro.getHeading().getDegrees());

        for (int index = 0; index < MODULE_COUNT; index++) {
            String moduleName = SwerveConstants.MODULE_CONFIGS.get(index).name();
            SwerveModuleState measured = modules[index].getState();
            SwerveModuleState desired = modules[index].getLastDesiredState();
            SmartDashboard.putNumber(
                "Swerve/" + moduleName + "/MeasuredSpeedMps",
                measured.speedMetersPerSecond);
            SmartDashboard.putNumber(
                "Swerve/" + moduleName + "/MeasuredAngleDegrees",
                measured.angle.getDegrees());
            SmartDashboard.putNumber(
                "Swerve/" + moduleName + "/DesiredSpeedMps",
                desired.speedMetersPerSecond);
            SmartDashboard.putNumber(
                "Swerve/" + moduleName + "/DesiredAngleDegrees",
                desired.angle.getDegrees());
            SmartDashboard.putNumber(
                "Swerve/" + moduleName + "/RawAbsoluteRotations",
                modules[index].getRawAbsolutePositionRotations());
        }
    }

    @Override
    public void close() {
        stop();
        for (SwerveModule module : modules) {
            module.close();
        }
        gyro.close();
    }
}
