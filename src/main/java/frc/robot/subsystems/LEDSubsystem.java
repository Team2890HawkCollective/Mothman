// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

  AddressableLED m_LED;

  AddressableLEDBuffer m_Buffer;

  AddressableLEDBufferView m_bottomHalf;

  AddressableLEDBufferView m_topHalf;

  AddressableLEDBufferView m_wholeStrip;

  private boolean isSwerveLocked = false;

  LEDPattern hubActiveColor = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kTeal, Color.kMagenta);
  LEDPattern hubActiveBlinkPattern = hubActiveColor.breathe(Second.of(0.4));

  LEDPattern hubInactiveColor = LEDPattern.solid(Color.kMagenta);
  LEDPattern hubInactiveBlinkPattern = hubInactiveColor.blink(Second.of(0.1));

  LEDPattern allianceShiftColor = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kTeal, Color.kMagenta);
  LEDPattern allianceShiftPattern = allianceShiftColor.scrollAtRelativeSpeed(Percent.per(Second).of(100));

  LEDPattern transitionColor = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kTeal, Color.kMagenta);
  LEDPattern transitionBlinkPattern = transitionColor.blink(Second.of(0.2));

  LEDPattern endGameColor = LEDPattern.solid(Color.kAquamarine);
  LEDPattern endGameBlinkPattern = endGameColor.blink(Second.of(0.175));

  LEDPattern rainbow = LEDPattern.rainbow(255, 128);
  LEDPattern rainbowScroll = rainbow.scrollAtRelativeSpeed(Percent.per(Second).of(100));

  LEDPattern lockPattern = LEDPattern.solid(Color.kRed);
  LEDPattern lockPatternBlink = lockPattern.blink(Second.of(.1));

  public LEDSubsystem() {
    m_LED = new AddressableLED(Constants.LEDConstants.LED_PWM_PORT);
    m_Buffer = new AddressableLEDBuffer(26);
    m_bottomHalf = m_Buffer.createView(0, 12);
    m_topHalf = m_Buffer.createView(13, 25);
    m_wholeStrip = m_Buffer.createView(0, 25);
    m_LED.setLength(m_Buffer.getLength());
    m_LED.setData(m_Buffer);
    m_LED.start();
  }

  private double matchTime;
  private boolean isHubActive;

  public void setLEDPeriod(AddressableLEDBufferView buffer) {
    if (DriverStation.isAutonomous()) {
      allianceShiftPattern.applyTo(buffer);
    }

    if (DriverStation.isTeleop()) {
      if (matchTime <= 140 && matchTime > 132) // transition
      {
        transitionBlinkPattern.applyTo(buffer);
      }

      else if (matchTime <= 132 && matchTime > 130) {
        hubInactiveBlinkPattern.applyTo(buffer);
      }

      else if (matchTime <= 107 && matchTime > 105 && isHubActive == false) {
        hubInactiveBlinkPattern.applyTo(buffer);
      }

      else if (matchTime <= 82 && matchTime > 80 && isHubActive == false) {
        hubInactiveBlinkPattern.applyTo(buffer);
      }

      else if (matchTime <= 57 && matchTime > 55 && isHubActive == false) {
        hubInactiveBlinkPattern.applyTo(buffer);
      }

      else if (matchTime <= 32 && matchTime > 30 && isHubActive == false) {
        hubInactiveBlinkPattern.applyTo(buffer);
      }

      else if (matchTime <= 30) // endgame
      {
        endGameBlinkPattern.applyTo(buffer);
      }

      else if (isHubActive == true) {
        hubActiveBlinkPattern.applyTo(buffer);
      }

      else {
        allianceShiftPattern.applyTo(buffer);
      }
    }
  }

  public void setLEDSwerveLocked(){
    lockPatternBlink.applyTo(m_bottomHalf);
  }

  public void setLEDLockedStatus(boolean status){
    isSwerveLocked = status;
  }

  public Command setLEDLockedStatusCommand(boolean status){
    return runOnce(()->setLEDLockedStatus(status));
  }



  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    matchTime = DriverStation.getMatchTime();
    isHubActive = isHubActive();

    if (isSwerveLocked == true) {
      setLEDPeriod(m_topHalf);
      setLEDSwerveLocked();
    }
    else {
      setLEDPeriod(m_wholeStrip);
    }

    m_LED.setData(m_Buffer);
  }
}
