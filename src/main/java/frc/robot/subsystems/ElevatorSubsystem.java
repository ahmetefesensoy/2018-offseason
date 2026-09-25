package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Drives the scissor-lift elevator to a target height using a single NEO. */
public class ElevatorSubsystem extends SubsystemBase {
    private static final double TOLERANCE_METERS = 0.02;
    private static final double MAX_VELOCITY_MPS = 1.0;
    private static final double MAX_ACCEL_MPS2 = 1.5;

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final ProfiledPIDController pid;
    private final ElevatorFeedforward feedforward;

    private double targetHeightMeters = Constants.ELEVATOR_GROUND_METERS;

    public ElevatorSubsystem() {
        motor = new SparkMax(Constants.ARM_MOTOR_PORT, MotorType.kBrushless);
        encoder = motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pid = new ProfiledPIDController(
            4.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(MAX_VELOCITY_MPS, MAX_ACCEL_MPS2));
        pid.setTolerance(TOLERANCE_METERS);

        feedforward = new ElevatorFeedforward(0.0, 0.3, 0.0);

        encoder.setPosition(0.0);
    }

    /** Commands the elevator to a new height, clamped to [GROUND, SCALE]. */
    public void setTargetHeight(double heightMeters) {
        targetHeightMeters = MathUtil.clamp(
            heightMeters,
            Constants.ELEVATOR_GROUND_METERS,
            Constants.ELEVATOR_SCALE_METERS);
        pid.setGoal(targetHeightMeters);
    }

    /** @return the elevator's current height in meters, derived from motor rotations. */
    public double getCurrentHeight() {
        return encoder.getPosition() * Constants.ELEVATOR_ROTATIONS_TO_METERS;
    }

    /** @return true once the elevator is within tolerance of its commanded target. */
    public boolean atTarget() {
        return Math.abs(getCurrentHeight() - targetHeightMeters) <= TOLERANCE_METERS;
    }

    public void stop() {
        motor.set(0.0);
    }

    /** Test-only accessor for the pending (clamped) target height. */
    double getTargetHeightForTest() {
        return targetHeightMeters;
    }

    /** Releases hardware handles; used by tests to clean up between cases. */
    void close() {
        motor.close();
    }

    @Override
    public void periodic() {
        double currentHeight = getCurrentHeight();
        double pidOutput = pid.calculate(currentHeight);
        double ffOutput = feedforward.calculate(pid.getSetpoint().velocity);
        motor.setVoltage(pidOutput + ffOutput);

        SmartDashboard.putNumber("Elevator Height (m)", currentHeight);
        SmartDashboard.putNumber("Elevator Target (m)", targetHeightMeters);
        SmartDashboard.putBoolean("Elevator At Target", atTarget());
    }
}
