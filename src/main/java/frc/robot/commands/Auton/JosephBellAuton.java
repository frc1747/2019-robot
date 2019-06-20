/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Drivetrain.DriveDistance;
import frc.robot.commands.Drivetrain.DriveProfiles;
import frc.robot.commands.Elevator.ElevatorProfiles;
import frc.robot.commands.HatchPanelScoring.ActuateHPScoring;
import frc.robot.commands.HatchPanelScoring.HPScore;
import frc.robot.subsystems.Elevator;
import lib.frc1747.commands.MakeParallel;
import lib.frc1747.commands.MakeSequential;

public class JosephBellAuton extends CommandGroup {

  public JosephBellAuton() {
    addSequential(new MakeSequential(
      new DriveProfiles("/home/lvuser/Joseph_Bell_test1_fwd_nor.csv"),
      new AutonTarget(1500),
      new ElevatorProfiles(2),
      new HPScore(),
      new ElevatorProfiles(1),

      new  DriveProfiles("/home/lvuser/Joseph_Bell_test2_rev_nor.csv"),

      new MakeParallel(
        new AutonTarget(2000),
        new ActuateHPScoring(751, true)
      ),
        
      new ActuateHPScoring(500, false),
      new DriveProfiles("/home/lvuser/backup_rev_nor.csv")
      // new DriveProfiles("/home/lvuser/Joseph_Bell_test3_rev_nor.csv"),

      // new AutonTarget(1500)
      //new HPScore(),

      //new DriveProfiles("/home/lvuser/Joseph_Bell_test4_rev_nor.csv")
    ));
  }
}
