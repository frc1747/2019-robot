/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  //TODO: We REALLY need to organize these...

  // invertion boolean values
  public static final boolean[] DRIVE_MOTOR_INVERT_LEFT = {false, false, false, false};
  public static final boolean[] DRIVE_MOTOR_INVERT_RIGHT = {true, true, false, true};
  public static final boolean LEFT_INTAKE_INVERTED = true;
  public static final boolean RIGHT_INTAKE_INVERTED = false;
  public static final boolean LEFT_ELEVATOR_INVERTED = true;
  public static final boolean RIGHT_ELEVATOR_INVERTED = true;


  // encoder scalings
  public static final double LEFT_ENCODER_SCALING = 1262.8 * 10.2 / 10;
  public static final double RIGHT_ENCODER_SCALING = 1280.8;
  public static final double ELEVATOR_SCALING = 468.58/4;

  // motor ports
  public static final int[] DRIVE_LEFT_MOTOR_PORTS = {1, 2, 3, 4};
  public static final int[] DRIVE_RIGHT_MOTOR_PORTS = {11, 12, 13, 14};

  public static final int LEFT_ELEVATOR_PORT = 21;
  public static final int RIGHT_ELEVATOR_PORT = 22;

  // public static final int LEFT_INTAKE_PORT = 34;  //TO BE DEPRECATED
  // public static final int RIGHT_INTAKE_PORT = 35; //TO BE DEPRECATED

  public static final int WRIST_TALON = 41;
  public static final boolean WRIST_INVERTED = false;
  public static final int ROLLER_TALON = 42;

  public static final int CARGO_SCORING_MOTOR_PORT = 31;
  public static final int CARGO_INTAKE_MOTOR_PORT = 32;

  public static final int DRIVER_PORT = 0;
  public static final int OPERATOR_PORT = 1;
  
  // rio ports
  public static final int ELEVATOR_ENCODER_PORT = 4;
  public static final int ELEVATOR_ENCODER_PORT_2 = 5;

  public static final int INTAKE_SENSOR_PORT = 2; //TO BE DEPRECATED
  // public static final int LEFT_IR_SENSOR_PORT = 6;
  // public static final int RIGHT_IR_SENSOR_PORT = 7;
  
  public static final int CARGO_SENSOR_PORT = 6;
  public static final int HATCH_INTAKE_ENCODER_PORT = 1;

  // solenoid ports
  public static final int HATCH_GRAB_SOLENOID = 0;
  public static final int CARGO_SOLENOID = 1;
  public static final int EXTENSION_SOLENOID = 2;

  // misc. constants
  public static final double DT = 0.02;
  public static final double S_V_MAX = 13;
  public static final double A_V_MAX = 4;
  public static final double SENSOR_THRESHOLD = 2.5; //check this value
  public static final double WRIST_ENCODER_GEAR = 18. / 28;
  public static final double WRIST_OFFSET = 2* Math.PI;
  public static final double GRAVITY_CONSTANT = 0.07;
  
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
