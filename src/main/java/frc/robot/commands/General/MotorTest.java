/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.General;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Auton.Delay;
import lib.frc1747.commands.MakeSequential;

public class MotorTest extends CommandGroup {

  public MotorTest() {
    addSequential(new MakeSequential(
        new MotorTestHelper(0, 0.5, 0.5, 2000),
        new Delay(2000),
        new MotorTestHelper(0, 0.5, -0.5, 2000),
        new Delay(2000),
        new MotorTestHelper(1, 0.5, 0.5, 2000),
        new Delay(2000),
        new MotorTestHelper(1, 0.5, -0.5, 2000),
        new Delay(2000),
        new MotorTestHelper(2, 0.5, 0.5, 2000),
        new Delay(2000),
        new MotorTestHelper(2, 0.5, -0.5, 2000),
        new Delay(2000),
        new MotorTestHelper(3, 0.5, 0.5, 2000),
        new Delay(2000),
        new MotorTestHelper(3, 0.5, -0.5, 2000)
      )
    );
  }
}