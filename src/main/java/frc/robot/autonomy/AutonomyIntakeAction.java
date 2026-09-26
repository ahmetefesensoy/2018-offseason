package frc.robot.autonomy;

/** Stable wire values shared with frc_autonomy_msgs/AutonomyCommand. */
public enum AutonomyIntakeAction {
    STOP(0),
    INTAKE(1),
    HOLD(2),
    EJECT(3);

    private final int code;

    AutonomyIntakeAction(int code) {
        this.code = code;
    }

    public int code() {
        return code;
    }

    public static AutonomyIntakeAction fromCode(long code) {
        for (AutonomyIntakeAction action : values()) {
            if (action.code == code) {
                return action;
            }
        }
        return null;
    }
}
