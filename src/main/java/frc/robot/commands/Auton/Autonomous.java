/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.Robot.AutonChoice;
import frc.robot.Robot.AutonPosition;
import frc.robot.commands.General.QuincysAttemptAtAuton;
import frc.robot.commands.General.QuincysAttemptAtAutonRight;

public class Autonomous extends CommandGroup {
  
  public Autonomous(AutonChoice auton, AutonPosition pos) {
    if(pos == Robot.AutonPosition.LEFT){
        if(auton == Robot.AutonChoice.JOSEPH_BELL_ROCKET){
          addSequential(new JosephBellAuton());
        }
        if(auton == Robot.AutonChoice.CENTER_SHIP){
          addSequential(new ShipAuton());
        }
        if (auton == Robot.AutonChoice.ROCKET_CARGO_COMBO){
          addSequential(new CloseRocketAndCargoShip());
        }
        if (auton == Robot.AutonChoice.SIDE_CARGO_SHIP){
          addSequential(new SideShipAuton());
        } 
        if (auton == Robot.AutonChoice.NATHAN_FASSNACHT_ROCKET){
          addSequential(new NathanFassnachtAuton());
        } 
        if (auton == Robot.AutonChoice.QUINCY_CARGO){
          addSequential(new QuincysAttemptAtAuton());
        } 
    }else if(pos == Robot.AutonPosition.RIGHT){
        if(auton == Robot.AutonChoice.JOSEPH_BELL_ROCKET){
          addSequential(new JosephBellAutonRight());
        }
        if(auton == Robot.AutonChoice.CENTER_SHIP){
          addSequential(new ShipAutonRight());
        }
        if (auton == Robot.AutonChoice.ROCKET_CARGO_COMBO){
          addSequential(new CloseRocketAndCargoShipRight());
        }
        if (auton == Robot.AutonChoice.SIDE_CARGO_SHIP){
          addSequential(new SideShipAutonRight());
        }
        if (auton == Robot.AutonChoice.NATHAN_FASSNACHT_ROCKET){
          addSequential(new NathanFassnachtAutonRight());
        } 
        if (auton == Robot.AutonChoice.QUINCY_CARGO){
          addSequential(new QuincysAttemptAtAutonRight());
        } 
    }
  }
}
