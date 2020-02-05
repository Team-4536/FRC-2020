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
    public TrajectoryFactory(double intX, double intY, double endX, double endY, double intTany,double intTanx,double endTany,double endTanx){ 
        TrenchyPathManager = new RamseteController(2.0,0.7);
    }
    
}