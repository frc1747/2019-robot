/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.General;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Auton.AutonTarget;
import frc.robot.commands.Drivetrain.DriveProfiles;
import frc.robot.commands.HatchPanelScoring.ActuateHPScoring;
import frc.robot.commands.HatchPanelScoring.HPScore;
import lib.frc1747.commands.MakeParallel;
import lib.frc1747.commands.MakeSequential;

public class QuincysAttemptAtAuton extends CommandGroup {
  /**
   * Add your docs here.
   */
  public QuincysAttemptAtAuton() {
    addSequential(new MakeSequential(
      new DriveProfiles("/home/lvuser/Quincys_Test_fwd_nor.csv"),
      new HPScore(),
      new DriveProfiles("/home/lvuser/Quincys_Test_2_rev_nor.csv"),
      new MakeParallel(
        new ActuateHPScoring(2000, true),
        new AutonTarget(2000)
      ),
      new ActuateHPScoring(500, false),
      new DriveProfiles("/home/lvuser/Quincys_Test_3_rev_nor.csv"),
      new AutonTarget(1750),
      new MakeParallel(
        new DriveProfiles("/home/lvuser/Quincys_Test_3_v2_rev_nor.csv"),
        new ActuateHPScoring(500, false)
    )));
  }
}
