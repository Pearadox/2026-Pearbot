// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

public class Launch extends Command {
  private final CANFuelSubsystem fuelSubsystem;

  public Launch(CANFuelSubsystem fuelSystem) {
    this.fuelSubsystem = fuelSystem;
    addRequirements(fuelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fuelSubsystem.setIntakeRoller(
        SmartDashboard.getNumber("Launching intake roller value", INTAKE_LAUNCHER_VOLTAGE));
    fuelSubsystem.setLauncherRoller(
        SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
    fuelSubsystem.setFeederRoller(
        SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
  }

  @Override
  public void execute() {
    // Continuous updates aren't needed for a basic voltage-based launch
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.stop(); 
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}