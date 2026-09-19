// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// Config imports
import com.revrobotics.spark.config.SparkFlexConfig;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  public static final class ShooterConstants {
    // Can Id Configs
    public static final int flywheelCanId1 = 0;
    public static final int flywheelCanId2 = 1;
    public static final int topFeederCanId = 2;

    // Flywheel PID configs
    public static final double kPflywheel = 0.0001; 
    public static final double kIflywheel = 0.0; 
    public static final double kDflywheel = 0.0; 

    // Flywheel feedforward config
    // kV: volts/rpm
    public static final double kVflywheel = 12.0/5767;

    public static final SparkFlexConfig flywheelMotorConfig = new SparkFlexConfig();
    public static final SparkFlexConfig topFeederMotorConfig = new SparkFlexConfig();


      static {
        flywheelMotorConfig
          .smartCurrentLimit(80)
          .idleMode(IdleMode.kCoast);
        
        flywheelMotorConfig
          .encoder
          .positionConversionFactor(1)
          .velocityConversionFactor(1);
        
          flywheelMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(kPflywheel, ClosedLoopSlot.kSlot1)
            .i(kIflywheel, ClosedLoopSlot.kSlot1)
            .d(kDflywheel, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
            .feedForward
              .kV(kVflywheel, ClosedLoopSlot.kSlot1);

          topFeederMotorConfig
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kCoast);
        
      }


  }
}
