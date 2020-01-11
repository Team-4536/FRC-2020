package frc4536.robot;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

public class TrajectoryFactory {
    double intX;
    double intY;
    double endX;
    double endY;
    double intTany;
    double intTanx;
    double endTany;
    double endTanx;
    double maxSpeed;
    double maxAcc;
    public TrajectoryFactory(double intX, double intY, double endX, double endY, double intTany,double intTanx,double endTany,double endTanx){ 
        this.intX = intX;
        this.intY = intY;
        this.endX = endX;
        this.endY = endY;
        this.intTanx = intTanx;
        this.intTany = intTany;
        this.endTanx = endTanx;
        this.endTany = endTany;
    }
    TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAcc);
            //.setKinematics(DriveConstants.kDriveKinematics)
            //.addConstraint(autoVoltageConstraint);

    Trajectory frontTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(intX, intY, new Rotation2d(Math.atan2(intTany, intTanx))),
    List.of(
        new Translation2d(4.280339775896371,-4.349554667197974),
        new Translation2d(6.500980548613312,-5.9819462285991705)),
    new Pose2d(endX,endY, new Rotation2d(Math.atan2(endTany, endTany))),
    config);
}