package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class DriveTrajectory extends SequentialCommandGroup {
    public DriveTrajectory(Trajectory trajectory, DriveTrain driveTrain, FileLog log){

        var thetaController =
            new ProfiledPIDController(
                Constants.TrajectoryConstants.kPThetaController, 0, 0, Constants.TrajectoryConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectory,
                driveTrain::getPose,
                Constants.DriveConstants.kDriveKinematics,
                new PIDController(Constants.TrajectoryConstants.kPXController, 0, 0),
                new PIDController(Constants.TrajectoryConstants.kPYController, 0, 0),
                thetaController,
                (a) -> driveTrain.setModuleStates(a, false),
                driveTrain);


        addCommands(
            new InstantCommand(() -> driveTrain.resetPose(trajectory.getInitialPose())),
            swerveControllerCommand,
            new DriveStop(driveTrain, log)
        );
    }
}