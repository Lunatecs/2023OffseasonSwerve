package frc.robot.utils;

import java.lang.String;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveDrive;

public class AutoFromPathPlanner extends SequentialCommandGroup {
  private PathPlannerTrajectory m_trajectory;

  public AutoFromPathPlanner(SwerveDrive drive, String pathName, double vel, double accel, boolean runReversed,
      boolean useAllianceColor,
      boolean endStationary) {
    m_trajectory = PathPlanner.loadPath(pathName, vel, accel, runReversed);

    var thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    AutoSwerveController swerveControllerCommand = new AutoSwerveController(m_trajectory, drive::getPose,
        Swerve.swerveKinematics, new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0), thetaController, drive::setModuleStates, useAllianceColor,
        drive);

    if (endStationary) {
      addCommands(
          swerveControllerCommand,
          new InstantCommand(() -> drive.stop()));
    } else {
      addCommands(
          swerveControllerCommand);
    }

  }

  public Pose2d getInitialPose() {
    return new Pose2d(m_trajectory.getInitialState().poseMeters.getX(),
        m_trajectory.getInitialState().poseMeters.getY(),
        m_trajectory.getInitialState().holonomicRotation.times(1.0));
  }

  public List<EventMarker> getEventMarkers() {
    return m_trajectory.getMarkers();
  }

  public PathPlannerState getInitialState() {
    return m_trajectory.getInitialState();
  }
  

}
