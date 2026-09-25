package frc.robot;

/** Static, compile-time robot constants. No instances. */
public final class Constants {
    private Constants() {}

    // DriverStation joystick slots are 0-5; the legacy Constant.java's value
    // of 8 was never valid (that file never compiled, so it was never caught).
    public static final int JOYSTICK_PORT = 0;

    // Intake roller motor CAN IDs (unchanged from legacy Constant.java)
    public static final int INTAKE_RIGHT_MOTOR_PORT = 2;
    public static final int INTAKE_LEFT_MOTOR_PORT = 1;

    // Elevator (scissor lift) motor CAN ID (was "kolun_motoru" in legacy Constant.java)
    public static final int ARM_MOTOR_PORT = 3;

    // TODO: gerçek donanım netleşince doğrulanacak — sonraki boş CAN ID.
    public static final int CLIMB_MOTOR_PORT = 9;

    // Elevator preset heights, meters. Source: 2018 FRC field manual
    // (Switch plate 9in/0.23m, Scale plate 5ft/1.52m at match start).
    public static final double ELEVATOR_GROUND_METERS = 0.0;
    public static final double ELEVATOR_SWITCH_METERS = 0.23;
    public static final double ELEVATOR_SCALE_METERS = 1.52;

    // TODO: kalibre et — gerçek scissor lift geometrisiyle ölçülecek.
    // Motor encoder tam turu ile lift yüksekliği (metre) arasındaki
    // dönüşüm katsayısı. 1.0 yalnızca derlenebilir bir placeholder'dır.
    public static final double ELEVATOR_ROTATIONS_TO_METERS = 1.0;
}
