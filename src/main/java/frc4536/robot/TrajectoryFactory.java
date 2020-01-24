package frc4536.robot;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

public class TrajectoryFactory {
    RamseteController TrenchyPathManager;
    public TrajectoryFactory(){ 
        TrenchyPathManager = new RamseteController(2.0,0.7);
    }
    
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    //TODO add rotation for each auto
    //collect from center goal
    Trajectory CenterShoot = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0.5913523322906798,-5.658436577910677, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(5.451690745623743,-7.57536394194789),
            new Translation2d(10.225458245742225,-7.612465761896998)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(12.909156555394325,-5.930516590870797, new Rotation2d(0)),
        // Pass config
        config
    );
    //collect from left of center of the goal
    Trajectory LeftCenterShoot = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0.653188698872526,-5.6708038512270456, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(5.315650739143684,-7.57536394194789),
            new Translation2d(10.2749273390077,-7.57536394194789)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(12.970992921976167,-3.7662437605062014, new Rotation2d(0)),
        // Pass config
        config
    );
    //collect from right of center of the goal
    Trajectory RightCenterShoot = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0.5913523322906798,-5.646069304594307, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(5.464058018940113,-7.600098488580628)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(12.995727468608909,-7.5877312152642595, new Rotation2d(0)),
        // Pass config
        config
    );

}