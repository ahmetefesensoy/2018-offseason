package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Skeleton subsystem for the hook/climb mechanism seen on the CAD
 * assembly. Real actuation hardware is not yet finalized; this exposes
 * only the minimal extend/retract/stop verbs. Not used in autonomous.
 */
public class ClimbSubsystem extends SubsystemBase {
    private static final double CLIMB_SPEED = 0.5;

    private final SparkMax motor;

    public ClimbSubsystem() {
        motor = new SparkMax(Constants.CLIMB_MOTOR_PORT, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void extend() {
        motor.set(CLIMB_SPEED);
    }

    public void retract() {
        motor.set(-CLIMB_SPEED);
    }

    public void stop() {
        motor.set(0.0);
    }
}
