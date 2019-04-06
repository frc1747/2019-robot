/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Drivetrain.DriveProfiles;
import frc.robot.commands.Extension.AutonExtend;
import frc.robot.commands.HatchPanelIntake.AutonHPIntake;
import frc.robot.commands.HatchPanelScoring.ActuateHPScoring;
import lib.frc1747.commands.MakeParallel;
import lib.frc1747.commands.MakeSequential;

public class SideShipAutonRight extends CommandGroup {

  public SideShipAutonRight() {

    addSequential(new MakeSequential(
      new  DriveProfiles("/home/lvuser/platform_to_close_side_cargo_fwd_mir.csv"),
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
      new  DriveProfiles("/home/lvuser/close_side_cargo_to_pickup_rev_mir.csv"),
      new  MakeParallel(
        new AutonTarget(1500),
        new ActuateHPScoring(1500, true)
      ),
      new DriveProfiles("/home/lvuser/pickup_to_center_cargo_rev_mir.csv"),
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
      )
      ));
  }
}
