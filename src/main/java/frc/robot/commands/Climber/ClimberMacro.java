/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Drivetrain.DriveDistance;
import frc.robot.commands.Extension.Extend;
import frc.robot.subsystems.Climber;
import lib.frc1747.commands.MakeParallel;
import lib.frc1747.commands.MakeSequential;

public class ClimberMacro extends CommandGroup {
  public ClimberMacro() {
    addSequential(
      new MakeSequential(
        new ClimberProfiles(Climber.LiftHeights.HAB3),
        new Extend(),
        new DriveDistance(10),
        new MakeParallel(
          new ClimberProfiles(Climber.LiftHeights.RETRACTED),
          new DriveDistance(5)
        )
      )
    );
  }
}
