package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Drivetrain state sampled on the roboRIO for safety and feedback. */
public record AutonomyRobotState(
        Pose2d pose,
        ChassisSpeeds measuredChassisSpeeds,
        boolean gyroHealthy,
        boolean drivetrainHealthy) {}
