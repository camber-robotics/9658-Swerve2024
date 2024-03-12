// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ClimbIOSparkMax implements ClimbIO {
  private static final double GEAR_RATIO = 1.5;

  private final CANSparkMax climbL = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax climbR = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax ramp = new CANSparkMax(11, MotorType.kBrushless);
  private final RelativeEncoder encoderL = climbL.getEncoder();
  private final SparkPIDController pidL = climbL.getPIDController();
  private final RelativeEncoder encoderR = climbR.getEncoder();
  private final SparkPIDController pidR = climbR.getPIDController();
  private final RelativeEncoder encoderRM = ramp.getEncoder();
  private final SparkPIDController pidRM = ramp.getPIDController();

  public ClimbIOSparkMax() {
    climbL.restoreFactoryDefaults();
    climbR.restoreFactoryDefaults();
    ramp.restoreFactoryDefaults();

    climbL.setCANTimeout(250);
    climbR.setCANTimeout(250);
    ramp.setCANTimeout(250);

    climbL.setInverted(false);
    climbR.setInverted(false);
    ramp.setInverted(false);

    climbL.enableVoltageCompensation(12.0);
    climbL.setSmartCurrentLimit(30);
    climbR.enableVoltageCompensation(12.0);
    climbR.setSmartCurrentLimit(30);
    ramp.enableVoltageCompensation(12.0);
    ramp.setSmartCurrentLimit(30);

    climbL.burnFlash();
    climbR.burnFlash();
    ramp.burnFlash();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoderL.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoderL.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = climbL.getAppliedOutput() * climbL.getBusVoltage();
    inputs.currentAmps = new double[] {climbL.getOutputCurrent(), climbR.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    climbL.setVoltage(volts);
    climbR.setVoltage(volts);
    ramp.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pidL.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
    pidR.setReference(
        Units.radiansPerSecondToRotationsPerMinute(-velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        1,
        -ffVolts,
        ArbFFUnits.kVoltage);
    pidRM.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        1,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    climbL.stopMotor();
    climbR.stopMotor();
    ramp.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pidL.setP(kP, 0);
    pidL.setI(kI, 0);
    pidL.setD(kD, 0);
    pidL.setFF(0, 0);

    pidR.setP(kP, 0);
    pidR.setI(kI, 0);
    pidR.setD(kD, 0);
    pidR.setFF(0, 0);

    pidRM.setP(kP, 0);
    pidRM.setI(kI, 0);
    pidRM.setD(kD, 0);
    pidRM.setFF(0, 0);
  }
}
