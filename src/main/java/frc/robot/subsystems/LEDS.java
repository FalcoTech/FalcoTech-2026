// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

/**
 * LED subsystem for the FalcoTech 2026 robot.
 *
 * <p>Drives a WS2812B-compatible addressable LED strip to convey two key states:
 *
 * <ul>
 *   <li><b>Shot likelihood</b> – a 0.0–1.0 scale supplied by external code (e.g. vision/targeting)
 *       that is rendered as a red→yellow→green gradient across the full strip.
 *   <li><b>Alliance shift period</b> – an alternating red/blue flash to alert drivers that the
 *       alliance-shift game period is active.
 * </ul>
 *
 * <p>All rendering happens inside {@link #periodic()} so callers only need to drive the mode via
 * the provided {@link Command} factory methods. The strip port and length are configured via {@link
 * #LED_PWM_PORT} and {@link #LED_LENGTH}.
 */
public class LEDS extends SubsystemBase {

  // ---- Hardware configuration ------------------------------------------------

  /** PWM port on the RoboRIO that the LED strip data line is wired to. */
  private static final int LED_PWM_PORT = 0;

  /** Number of individually addressable LEDs on the strip. */
  private static final int LED_LENGTH = 60;

  // ---- Animation timing ------------------------------------------------------

  /** How long each half-cycle of the alliance-shift flash lasts (seconds). */
  private static final double ALLIANCE_SHIFT_FLASH_PERIOD_S = 0.5;

  // ---- Internal state --------------------------------------------------------

  public enum LEDMode {
    OFF,
    ALLIANCE,
    SHOT_LIKELIHOOD,
    ALLIANCE_SHIFT
  }

  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;

  private LEDMode currentMode = LEDMode.OFF;

  /** Clamped [0.0, 1.0] value set by the shot-likelihood command each cycle. */
  private double shotLikelihood = 0.0;

  /** True = red alliance, false = blue alliance. */
  private boolean isRedAlliance = true;

  private final Timer animationTimer = new Timer();

  // ---- Constructor -----------------------------------------------------------

  public LEDS() {
    ledStrip = new AddressableLED(LED_PWM_PORT);
    ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
    ledStrip.setLength(LED_LENGTH);
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    animationTimer.start();
  }

  // ---- Periodic --------------------------------------------------------------

  @Override
  public void periodic() {
    switch (currentMode) {
      case OFF:
        applyOff();
        break;
      case ALLIANCE:
        applyAllianceColor();
        break;
      case SHOT_LIKELIHOOD:
        applyShotLikelihoodColor();
        break;
      case ALLIANCE_SHIFT:
        applyAllianceShiftAnimation();
        break;
    }
    ledStrip.setData(ledBuffer);
  }

  // ---- Public API ------------------------------------------------------------

  /**
   * Informs the subsystem of the current alliance so that {@link #showAlliance()} and the
   * post-alliance-shift restore display the correct color.
   *
   * @param isRed {@code true} for red alliance, {@code false} for blue.
   */
  public void setAlliance(boolean isRed) {
    isRedAlliance = isRed;
  }

  // ---- Command factories -----------------------------------------------------

  /**
   * Returns a command that <em>continuously</em> reads a shot-likelihood value from the supplied
   * function and updates the LED strip color accordingly.
   *
   * <p>The color transitions smoothly from red (0.0 – no chance) through yellow (0.5 – even odds)
   * to green (1.0 – certain). Intended to run as the default command while the robot is targeting
   * the Hub, or whenever shot-readiness feedback is desired.
   *
   * @param likelihoodSupplier provides the current shot-likelihood in [0.0, 1.0]
   */
  public Command showShotLikelihood(DoubleSupplier likelihoodSupplier) {
    return run(
        () -> {
          shotLikelihood = Math.max(0.0, Math.min(1.0, likelihoodSupplier.getAsDouble()));
          currentMode = LEDMode.SHOT_LIKELIHOOD;
        });
  }

  /**
   * Returns a command that shows an alternating red/blue flash for the duration of the
   * alliance-shift game period. When the command ends (cancelled or timed out by the caller), the
   * strip reverts to the robot's alliance color.
   *
   * <p>Example usage in {@code RobotContainer}:
   *
   * <pre>{@code
   * // Trigger tied to the alliance-shift period flag supplied by game state logic:
   * allianceShiftTrigger.whileTrue(leds.allianceShiftIndicator());
   * }</pre>
   */
  public Command allianceShiftIndicator() {
    return startEnd(
        () -> {
          currentMode = LEDMode.ALLIANCE_SHIFT;
          animationTimer.reset();
        },
        () -> currentMode = LEDMode.ALLIANCE);
  }

  /**
   * Returns a command that sets the strip to the robot's alliance color (solid red or blue).
   * Runs once; the color persists until another command changes the mode.
   */
  public Command showAlliance() {
    return runOnce(() -> currentMode = LEDMode.ALLIANCE);
  }

  /** Returns a command that turns the entire strip off. Runs once. */
  public Command off() {
    return runOnce(() -> currentMode = LEDMode.OFF);
  }

  // ---- Private rendering helpers ---------------------------------------------

  private void applyOff() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 0);
    }
  }

  private void applyAllianceColor() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (isRedAlliance) {
        ledBuffer.setRGB(i, 200, 0, 0);
      } else {
        ledBuffer.setRGB(i, 0, 0, 200);
      }
    }
  }

  /**
   * Maps {@link #shotLikelihood} → WPILib HSV hue and applies it to every pixel.
   *
   * <p>WPILib uses a 0–180 hue range (half the standard 0–360):
   *
   * <ul>
   *   <li>Hue 0 → red (low likelihood)
   *   <li>Hue 30 → yellow (medium likelihood)
   *   <li>Hue 60 → green (high likelihood)
   * </ul>
   */
  private void applyShotLikelihoodColor() {
    int hue = (int) (shotLikelihood * 60); // 0 (red) → 60 (green)
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, hue, 255, 200);
    }
  }

  /** Flashes the strip between red and blue at {@link #ALLIANCE_SHIFT_FLASH_PERIOD_S} intervals. */
  private void applyAllianceShiftAnimation() {
    double elapsed = animationTimer.get();
    boolean showRed = (int) (elapsed / ALLIANCE_SHIFT_FLASH_PERIOD_S) % 2 == 0;
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (showRed) {
        ledBuffer.setRGB(i, 200, 0, 0);
      } else {
        ledBuffer.setRGB(i, 0, 0, 200);
      }
    }
  }
}
