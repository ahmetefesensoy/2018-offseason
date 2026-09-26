package frc.robot.subsystems.swerve;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.SwerveConstants;
import frc.robot.SwerveConstants.ModuleConfig;

/** Real NEO/SparkMax and Through Bore Encoder implementation. */
public final class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkAbsoluteEncoder absoluteEncoder;
    private final double absoluteOffsetRadians;
    private final boolean configured;

    public SwerveModuleIOSparkMax(ModuleConfig moduleConfig) {
        driveMotor = new SparkMax(moduleConfig.driveCanId(), MotorType.kBrushless);
        turnMotor = new SparkMax(moduleConfig.turnCanId(), MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        absoluteEncoder = turnMotor.getAbsoluteEncoder();
        absoluteOffsetRadians = moduleConfig.absoluteOffsetRadians();

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.inverted(moduleConfig.driveInverted());
        driveConfig.smartCurrentLimit(SwerveConstants.DRIVE_CURRENT_LIMIT_AMPS);
        driveConfig.voltageCompensation(SwerveConstants.NOMINAL_VOLTAGE);
        driveConfig.encoder.positionConversionFactor(
            SwerveConstants.DRIVE_POSITION_FACTOR_METERS);
        driveConfig.encoder.velocityConversionFactor(
            SwerveConstants.DRIVE_VELOCITY_FACTOR_MPS);

        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig.idleMode(IdleMode.kBrake);
        turnConfig.inverted(moduleConfig.turnInverted());
        turnConfig.smartCurrentLimit(SwerveConstants.TURN_CURRENT_LIMIT_AMPS);
        turnConfig.voltageCompensation(SwerveConstants.NOMINAL_VOLTAGE);
        turnConfig.absoluteEncoder.setSparkMaxDataPortConfig();
        turnConfig.absoluteEncoder.inverted(moduleConfig.absoluteEncoderInverted());
        turnConfig.absoluteEncoder.positionConversionFactor(
            SwerveConstants.TURN_POSITION_FACTOR_RADIANS);
        turnConfig.absoluteEncoder.velocityConversionFactor(
            SwerveConstants.TURN_VELOCITY_FACTOR_RAD_PER_SEC);

        REVLibError driveConfigurationResult = driveMotor.configure(
            driveConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        REVLibError turnConfigurationResult = turnMotor.configure(
            turnConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        configured = driveConfigurationResult == REVLibError.kOk
            && turnConfigurationResult == REVLibError.kOk;
    }

    @Override
    public Inputs readInputs() {
        double rawAngleRadians = absoluteEncoder.getPosition();
        double calibratedAngleRadians = MathUtil.angleModulus(
            rawAngleRadians - absoluteOffsetRadians);
        return new Inputs(
            driveEncoder.getPosition(),
            driveEncoder.getVelocity(),
            Rotation2d.fromRadians(calibratedAngleRadians),
            rawAngleRadians / (2.0 * Math.PI));
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnMotor.setVoltage(volts);
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }

    @Override
    public boolean isHealthy() {
        return configured
            && !driveMotor.hasActiveFault()
            && !turnMotor.hasActiveFault()
            && driveMotor.getLastError() == REVLibError.kOk
            && turnMotor.getLastError() == REVLibError.kOk;
    }

    @Override
    public void close() {
        driveMotor.close();
        turnMotor.close();
    }
}
