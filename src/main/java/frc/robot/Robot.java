// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

//FIXME: Add full logging functionality.
public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  private PowerDistribution powerDistribution;

  private boolean teleop = false;

  public Robot() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "MyProject");
    if(isReal()){
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      powerDistribution = new PowerDistribution(Constants.PDH_ID, ModuleType.kRev);
    }
    else{
      powerDistribution = new PowerDistribution(Constants.PDH_ID, ModuleType.kRev);
    }
    if(Constants.DEBUG){
      Logger.start();
    }

    powerDistribution.setSwitchableChannel(true);
    powerDistribution.clearStickyFaults();

    PathfindingCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    if(DriverStation.isFMSAttached() && DriverStation.isTeleopEnabled()){
      teleop = true;
    }
  }

  @Override
  public void disabledInit() {
    if(DriverStation.isEStopped() || (DriverStation.isFMSAttached() && DriverStation.getMatchTime() < 3 && teleop)){
      //robotContainer.getMusicCommand().ignoringDisable(true).schedule();
    }
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if(autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if(autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}

































































































































































































































/*
 * Beavers to inches = 36 / number of beaver
 */




// Luke is Coach's Teddy Bear 





//smidgen = 0.1
/*if(DriverStation.isDisabled() && flag && DriverStation.getMatchTime() > 130){
  //FIX ME Get RoboRandy and Greyson to Approve!
  flag = false;
  Shuffleboard.getTab("GREYSON").add("GREYSON", "STRONGER CORE!!  MORE SIX-PACK!! BEST LIFT!!").withPosition(0, 0);
  Shuffleboard.selectTab("GREYSON");
}
if(DriverStation.isEStopped() && flag1){
  //FIX ME Get RoboRandy to Approve!
  flag1 = false;
  Shuffleboard.getTab("Death").add("Death", "We have failed!!! :(").withPosition(0, 0);
  Shuffleboard.getTab("Death").add("Death2", "MARKKKKKKKK!!!!!!").withPosition(0, 1);
  Shuffleboard.selectTab("Death");
}*/
//This is the life of a programmer. We are under valued and over worked. Everything we get breaks. There are so many mechanical issues with the robot that I have lost count. This will probably never get seen again, but if it does, good luck in your future in Java. I hope that you don't have to use Shuffleboard.
//Sincerely, 
//  Jack Left