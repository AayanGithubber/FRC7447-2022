// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  WPI_VictorSPX m_intakeMotor;

  /** Creates a new Intake. */
  public Intake() {
    m_intakeMotor = new WPI_VictorSPX(Constants.intakeM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeBall(double speed) {
    m_intakeMotor.set(speed);
  }

  public void stop() {
    m_intakeMotor.set(0);
  }
}
