package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Swerve.*;

public class IO extends SubsystemBase {
        final CommandXboxController driveController = new CommandXboxController(0);

        public final Swerve chassis = new Swerve();

        public CommandScheduler scheduler = CommandScheduler.getInstance();

        public IO(SendableChooser<Runnable> bindings) {
                // bindings.addOption("Manual", this::configManual); Work on manual later
        }

        public void configGlobal() {
                chassis.setDefaultCommand(new DefaultDrive(this, driveController));

                driveController.leftBumper()
                                .onTrue(new InstantCommand(() -> chassis.DRIVE_MODE = DriveConstants.ROBOT_ORIENTED));
                driveController.rightBumper()
                                .onTrue(new InstantCommand(() -> chassis.DRIVE_MODE = DriveConstants.FIELD_ORIENTED));
                driveController.leftTrigger().onTrue(new InstantCommand(() -> chassis.SPEED_TYPE = DriveConstants.SLOW))
                                .onFalse(new InstantCommand(() -> chassis.SPEED_TYPE = 0));
                driveController.rightTrigger()
                                .onTrue(new InstantCommand(() -> chassis.SPEED_TYPE = DriveConstants.TURBO))
                                .onFalse(new InstantCommand(() -> chassis.SPEED_TYPE = 0));

                driveController.back().onTrue(new InstantCommand(chassis::resetOdometry));

                DriverStation.silenceJoystickConnectionWarning(true);
        }


        StructPublisher<Pose2d> estimated_pose = NetworkTableInstance.getDefault().getTable("Debug")
                        .getStructTopic("Estimated Pose", Pose2d.struct).publish();

        @Override
        public void periodic() {

        }
}