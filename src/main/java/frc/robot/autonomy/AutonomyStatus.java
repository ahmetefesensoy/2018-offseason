package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Observable roboRIO state returned to ROS 2 through NT4. */
public record AutonomyStatus(
        String sessionId,
        String mode,
        long acceptedSequence,
        long robotTimeMicros,
        long commandAgeMicros,
        boolean commandActive,
        boolean gyroHealthy,
        boolean drivetrainHealthy,
        AutonomyRejectReason rejectReason,
        Pose2d pose,
        ChassisSpeeds measuredChassisSpeeds) {}
