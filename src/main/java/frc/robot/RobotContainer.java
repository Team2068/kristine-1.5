// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.DriveConstants;
import frc.robot.utility.IO;

public class RobotContainer {
  SendableChooser<Runnable> bindings = new SendableChooser<Runnable>();
  SendableChooser<Command> autos;
  HashMap<String, Command> commands = new HashMap<>();

  public IO io = new IO(bindings);

  public RobotContainer() {
    // addAutos();

    autos = AutoBuilder.buildAutoChooser("Speaker Rings Centre");
    // autos.addOption("Adaptive Path", new AdaptivePath());
 
    SmartDashboard.putData("Autos",autos);
    SmartDashboard.putData("Bindings", bindings);
    SmartDashboard.putData("Autonomous", new SequentialCommandGroup(
      new InstantCommand(() -> io.chassis.DRIVE_MODE = DriveConstants.FIELD_ORIENTED),
      new InstantCommand(autos.getSelected()::schedule)));

      io.configGlobal();
  }

  // public void addAutos() {
  //   NamedCommands.registerCommands(commands);
  // }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> io.chassis.DRIVE_MODE = DriveConstants.FIELD_ORIENTED),
        autos.getSelected());
  }
}