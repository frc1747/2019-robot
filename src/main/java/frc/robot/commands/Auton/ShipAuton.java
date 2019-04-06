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
import frc.robot.commands.Extension.AutonExtend;
import lib.frc1747.commands.MakeParallel;
import lib.frc1747.commands.MakeSequential;


public class ShipAuton extends CommandGroup {

  public ShipAuton() {
    addSequential(new MakeSequential(
      new DriveProfiles("/home/lvuser/platform_to_center_cargo_fwd_nor.csv"),
      new MakeParallel(  
        new AutonExtend(250, true),
        new MakeSequential(
          new Delay(100),
          new ActuateHPScoring(250, true)
        )
      ),
      new MakeSequential(
        new Delay(125),
        new AutonExtend(250, false)
      ),

      new DriveProfiles("/home/lvuser/cargo_ship_to_feeder_station_rev_nor.csv"),
      new MakeParallel(
        new AutonTarget(1750),
        new ActuateHPScoring(750, true)
      ),

      new ActuateHPScoring(150, false),

      new DriveProfiles("/home/lvuser/test3_rev_nor.csv")
      /*
      new MakeParallel(  
        new AutonExtend(250, true),
        new MakeSequential(
          new Delay(100),
          new ActuateHPScoring(250, true)
        )
      ),
      new MakeSequential(
        new Delay(125),
        new AutonExtend(250, false)
      )*/
    ));
  }
}