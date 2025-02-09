// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbAvator.ClimbAvator;
import frc.robot.subsystems.ClimbAvator.ClimbAvatorStates;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MovementCommand extends SequentialCommandGroup {
  private ClimbAvator s_ClimbAvator;
  /** Creates a new MovementCommand. */
  public MovementCommand(ClimbAvator s_ClimbAvator, ClimbAvatorStates state) {
    this.s_ClimbAvator = s_ClimbAvator;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(Commands.runOnce(()->System.out.println("StateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateState "+ClimbAvatorStates.STATION.getAngle())), s_ClimbAvator.setShoulder());
  }

  private Command test() {

    return null;//Commands.defer(() -> { System.out.println("StateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateStateState "+ClimbAvatorStates.STATION.getAngle());
            //                      return s_ClimbAvator.setShoulder(() -> ClimbAvatorStates.STATION);}, Set.of(s_ClimbAvator));
  }
}
