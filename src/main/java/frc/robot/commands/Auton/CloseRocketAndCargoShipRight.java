/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Drivetrain.DriveProfiles;
import frc.robot.commands.HatchPanelScoring.ActuateHPScoring;
import frc.robot.commands.HatchPanelScoring.HPScore;
import lib.frc1747.commands.MakeParallel;
import lib.frc1747.commands.MakeSequential;

public class CloseRocketAndCargoShipRight extends CommandGroup {

  public CloseRocketAndCargoShipRight() {
    addSequential(new MakeSequential(
      new  DriveProfiles("/home/lvuser/Joseph_Bell_test1_fwd_mir.csv"),
      new AutonTarget(1500),
      new  HPScore(),

      new  DriveProfiles("/home/lvuser/Joseph_Bell_test2_rev_mir.csv"),

      new MakeParallel(
        new AutonTarget(1750),
        new ActuateHPScoring(751, true)
      ),
        
      new ActuateHPScoring(150, false),
      new DriveProfiles("/home/lvuser/pickup_to_close_cargo_rev_mir.csv")
    ));
  }
}
