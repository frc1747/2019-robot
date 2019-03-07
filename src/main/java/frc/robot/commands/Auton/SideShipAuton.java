/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Drivetrain.DriveProfiles;
import frc.robot.commands.HatchPanelScoring.HPScore;
import lib.frc1747.commands.MakeSequential;

public class SideShipAuton extends CommandGroup {
  /**
   * Add your docs here.
   */
  public SideShipAuton() {
    addSequential(new MakeSequential(
      new  DriveProfiles("/home/lvuser/platform_to_side_cargo_1_fwd_nor.csv"),
      new HPScore()));
      // new  DriveProfiles("/home/lvuser/center_cargo_to_center_rev_nor.csv")
      // new  DriveProfiles("/home/lvuser/center_to_cargos_fwd_nor.csv")
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
