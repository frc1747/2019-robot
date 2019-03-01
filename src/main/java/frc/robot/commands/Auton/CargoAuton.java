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
import lib.frc1747.commands.MakeParallel;
import lib.frc1747.commands.MakeSequential;
import frc.robot.commands.Extension.AutonExtend;


public class CargoAuton extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CargoAuton() {
    addSequential(new MakeSequential(
      new MakeSequential(
        new DriveProfiles("/home/lvuser/platform_to_side_cargo_1_fwd_nor.csv"),
        new MakeParallel(
          // new AutonExtend(100),
          new ActuateHPScoring(200, true)
        ),
      new DriveProfiles("/home/lvuser/center_to_pickup_2_fwd_nor.csv"),
      new ActuateHPScoring(200, false)
        ),
        new DriveProfiles("/home/lvuser/pickup_to_center_far_fwd_nor.csv"),
        new DriveProfiles("/home/lvuser/center_far_to_side_cargo_2_fwd_nor.csv"),
        new MakeParallel(
          // new AutonExtend(100),
          new ActuateHPScoring(200, true)
        )
    ));
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
