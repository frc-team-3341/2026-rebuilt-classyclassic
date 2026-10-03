// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = Mode.REAL;
  // RobotBase.isReal() ? Mode.REAL : simMode;

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
    public static final int CANIDCONSTANTFLYWHEEL1 = 0;
    public static final int CANIDCONSTANTFLYWHEEL2 = 1;
    public static final int CANIDCONSTANTTOPFEEDER = 2;

    // Flywheel PID configs
    public static final double kPflywheel = 0.0001;
    public static final double kIflywheel = 0.0;
    public static final double kDflywheel = 0.0;

    // Feeder PID configs
    public static final double kPfeeder = 0.0001;
    public static final double kIfeeder = 0.0;
    public static final double kDfeeder = 0.0;

    // Feedforward config
    // kV: volts/rpm
    public static final double kVflywheel = 12.0 / 5767;
    public static final double kVfeeder = 12.0 / 5767;

    // Motor Configs
    public static final SparkFlexConfig FLYWHEELMOTORCONFIG = new SparkFlexConfig();
    public static final SparkFlexConfig FOLLOWERMOTORCONFIG = new SparkFlexConfig();
    public static final SparkFlexConfig FEEDERMOTORCONFIG = new SparkFlexConfig();

    // Feeder Configs
    public static final double FEEDERRPM = 500;

    // Reverse Flywheel
    public static final boolean REVERSEFLYWHEEL = false;

    // Flywheel configs
    public static final double BACKUPSHOOTERRPM = 3000;
    public static final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();
    
    static {
      // Update Shooter Tree Map
      // Key: distance (in meters) 
      // Value: flywheel RPM
      shooterMap.put(0.0, 0.0);
    }

    static {
      // Configure Motors.
      FLYWHEELMOTORCONFIG.smartCurrentLimit(80).idleMode(IdleMode.kCoast);

      FLYWHEELMOTORCONFIG.encoder.positionConversionFactor(1).velocityConversionFactor(1);

      FLYWHEELMOTORCONFIG
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(kPflywheel, ClosedLoopSlot.kSlot1)
          .i(kIflywheel, ClosedLoopSlot.kSlot1)
          .d(kDflywheel, ClosedLoopSlot.kSlot1)
          .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
          .feedForward
          .kV(kVflywheel, ClosedLoopSlot.kSlot1);
      
      FOLLOWERMOTORCONFIG.follow(CANIDCONSTANTFLYWHEEL1,true);

      FEEDERMOTORCONFIG.smartCurrentLimit(80).idleMode(IdleMode.kBrake);

      FEEDERMOTORCONFIG.encoder.positionConversionFactor(1).velocityConversionFactor(1);

      FEEDERMOTORCONFIG
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(kPfeeder, ClosedLoopSlot.kSlot1)
          .i(kIfeeder, ClosedLoopSlot.kSlot1)
          .d(kDfeeder, ClosedLoopSlot.kSlot1)
          .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
          .feedForward
          .kV(kVfeeder, ClosedLoopSlot.kSlot1);
    }
  }
}
