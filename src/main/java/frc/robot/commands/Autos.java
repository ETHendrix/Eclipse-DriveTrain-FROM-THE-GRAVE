// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Yoshi;
import edu.wpi.first.wpilibj2.command.CommandBase;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(Yoshi subsystem) {
    return null;
    //Commands.sequence(subsystem.RunInFeed(), new Infeed());
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
