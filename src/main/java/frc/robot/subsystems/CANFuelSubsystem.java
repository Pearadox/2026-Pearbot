// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final SparkMax intakeRoller;
  private final SparkFlex launcherRoller;

  /** Creates a new CANFuelSubsystem. */
  public CANFuelSubsystem() {
    // Initialize motors
    intakeRoller = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    launcherRoller = new SparkFlex(LAUNCHER_MOTOR_ID, MotorType.kBrushless);

    // Feeder Motor Configuration
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Launcher Motor 1 Configuration (SparkMax)
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
    intakeRoller.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Launcher Motor 2 Configuration (SparkFlex)
    SparkFlexConfig launcherconfig = new SparkFlexConfig();
    launcherconfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherRoller.configure(launcherconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Dashboard tuning values
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching intake roller value", INTAKE_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);

  }

  // Method to set voltage for the intake/launcher motor
  public void setIntakeRoller(double voltage) {
    intakeRoller.setVoltage(voltage);
  }

  // Method to set voltage for the feeder motor
  public void setFeederRoller(double voltage) {
    feederRoller.setVoltage(voltage);
  }

  // Method to set voltage for the second launcher motor (Spark Flex)
  public void setLauncherRoller(double voltage) {
    launcherRoller.setVoltage(voltage);
  }

  // Method to stop all rollers
  public void stop() {
    feederRoller.set(0);
    intakeRoller.set(0);
    launcherRoller.setVoltage(IDLE_LAUNCHER_VOLTAGE); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}