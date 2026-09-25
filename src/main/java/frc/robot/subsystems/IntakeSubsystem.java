package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Controls the two-sided, 6-roller box gripper: rollers close inward to grip
 * a Power Cube and open outward to release it. Behavior ported unchanged
 * from the original Intake.java.
 */
public class IntakeSubsystem extends SubsystemBase {
    private static final double OPEN_LIMIT = 0.20;
    private static final double DRIVE_SPEED = 0.2;
    private static final double CUBE_RANGE_CM = 2.0;

    private final SparkMax rightMotor;
    private final SparkMax leftMotor;
    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;
    private final Ultrasonic cubeSensor;

    private double rangeCm;

    public IntakeSubsystem() {
        rightMotor = new SparkMax(Constants.INTAKE_RIGHT_MOTOR_PORT, MotorType.kBrushed);
        leftMotor = new SparkMax(Constants.INTAKE_LEFT_MOTOR_PORT, MotorType.kBrushed);

        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        cubeSensor = new Ultrasonic(1, 2);
        Ultrasonic.setAutomaticMode(true);

        resetEncoders();
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    /**
     * Closes the rollers inward to grip a cube; stops each side once closed.
     * Open position is right=-OPEN_LIMIT/left=+OPEN_LIMIT; closed is 0 for
     * both, so closing must drive back toward 0 from whichever side open()
     * left the encoder on.
     */
    public void close() {
        if (rightEncoder.getPosition() < 0) {
            rightMotor.set(DRIVE_SPEED);
        } else {
            rightMotor.set(0.0);
        }

        if (leftEncoder.getPosition() > 0) {
            leftMotor.set(-DRIVE_SPEED);
        } else {
            leftMotor.set(0.0);
        }
    }

    /** Opens the rollers outward to release a cube; stops each side at the limit. */
    public void open() {
        if (rightEncoder.getPosition() > -OPEN_LIMIT) {
            rightMotor.set(-DRIVE_SPEED);
        } else {
            rightMotor.set(0.0);
        }

        if (leftEncoder.getPosition() < OPEN_LIMIT) {
            leftMotor.set(DRIVE_SPEED);
        } else {
            leftMotor.set(0.0);
        }
    }

    /** @return true if the ultrasonic sensor reports a cube within grip range. */
    public boolean hasCube() {
        return cubeDetected(cubeSensor.isRangeValid(), rangeCm);
    }

    /**
     * Pure decision logic for hasCube(), split out for testability.
     * getRangeInches() returns 0 when no echo has been measured yet, which
     * would otherwise read as "cube present" (0 < CUBE_RANGE_CM) on every
     * disabled or just-booted robot; isRangeValid must gate that.
     */
    static boolean cubeDetected(boolean isRangeValid, double rangeCm) {
        return isRangeValid && rangeCm < CUBE_RANGE_CM;
    }

    /** Closes automatically when a cube is sensed, opens otherwise. */
    public void autoIntake() {
        if (hasCube()) {
            close();
        } else {
            open();
        }
    }

    public void stop() {
        rightMotor.set(0.0);
        leftMotor.set(0.0);
    }

    /** Test-only accessor for reading the right motor's last commanded output. */
    SparkMax getRightMotorForTest() {
        return rightMotor;
    }

    /** Test-only accessor for reading the left motor's last commanded output. */
    SparkMax getLeftMotorForTest() {
        return leftMotor;
    }

    /** Releases hardware handles; used by tests to clean up between cases. */
    void close_() {
        rightMotor.close();
        leftMotor.close();
        cubeSensor.close();
    }

    @Override
    public void periodic() {
        rangeCm = cubeSensor.getRangeInches() * 2.54;
        SmartDashboard.putNumber("Intake Mesafe (cm)", rangeCm);
        SmartDashboard.putBoolean("Intake Kutu Var Mi", hasCube());
    }
}
