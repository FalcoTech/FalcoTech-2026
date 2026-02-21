// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

/**
 * LED subsystem driven by a REV Blinkin LED Driver.
 *
 * <p>The Blinkin accepts a PWM signal (via a {@link Spark}) and selects its pattern/color from a
 * lookup table based on the set-point value. All set-point constants below are taken from the REV
 * Blinkin LED Driver pattern guide — verify them against the chart included with your unit.
 *
 * <p>Two primary display modes are provided:
 *
 * <ul>
 *   <li><b>Shot likelihood</b> – a 0.0–1.0 scale supplied by external targeting/vision code,
 *       rendered as a smooth red→yellow→green color progression through Blinkin solid-color
 *       set-points.
 *   <li><b>Alliance shift period</b> – an alternating red/blue flash so drivers immediately
 *       recognise the alliance-shift game period.
 * </ul>
 */
public class LEDS extends SubsystemBase {

  // ---- Hardware configuration ------------------------------------------------

  /** PWM port on the RoboRIO the Blinkin signal wire is connected to. */
  private static final int BLINKIN_PWM_PORT = Constants.BLINKIN_PWM_PORT;

  // ---- Blinkin solid-color set-points ----------------------------------------
  // Values from the REV Blinkin LED Driver pattern guide (solid colors section).
  // Solid colors run in 0.02 increments from ~0.57 (Hot Pink) up through 0.99 (Black).

  private static final double COLOR_RED    = 0.61;
  private static final double COLOR_YELLOW = 0.69;
  private static final double COLOR_GREEN  = 0.77;
  private static final double COLOR_BLUE   = 0.87;
  /** Black / off — Blinkin lowest-intensity solid. */
  private static final double COLOR_OFF    = 0.99;

  // ---- Shot-likelihood display -----------------------------------------------

  /**
   * PWM set-point for 0 % shot likelihood (red).
   * Interpolation walks from here toward {@link #LIKELIHOOD_HIGH_PWM}.
   */
  private static final double LIKELIHOOD_LOW_PWM  = COLOR_RED;

  /**
   * PWM set-point for 100 % shot likelihood (green).
   * Because the Blinkin's solid-color section is strictly monotone between red and green
   * we can linearly interpolate through orange→yellow→lime on the way.
   */
  private static final double LIKELIHOOD_HIGH_PWM = COLOR_GREEN;

  // ---- Animation timing ------------------------------------------------------

  /** Duration of each red or blue half-flash during the alliance-shift animation (seconds). */
  private static final double ALLIANCE_SHIFT_FLASH_PERIOD_S = 0.5;

  // ---- Internal state --------------------------------------------------------

  public enum LEDMode {
    OFF,
    ALLIANCE,
    SHOT_LIKELIHOOD,
    ALLIANCE_SHIFT
  }

  private final Spark blinkin;

  private LEDMode currentMode = LEDMode.OFF;
  private double shotLikelihood = 0.0;
  private boolean isRedAlliance = true;
  private final Timer animationTimer = new Timer();

  // ---- Constructor -----------------------------------------------------------

  public LEDS() {
    blinkin = new Spark(BLINKIN_PWM_PORT);
    animationTimer.start();
  }

  // ---- Periodic --------------------------------------------------------------

  @Override
  public void periodic() {
    switch (currentMode) {
      case OFF:
        blinkin.set(COLOR_OFF);
        break;
      case ALLIANCE:
        blinkin.set(isRedAlliance ? COLOR_RED : COLOR_BLUE);
        break;
      case SHOT_LIKELIHOOD:
        blinkin.set(likelihoodToPWM(shotLikelihood));
        break;
      case ALLIANCE_SHIFT:
        applyAllianceShiftAnimation();
        break;
    }
  }

  // ---- Public API ------------------------------------------------------------

  /**
   * Stores the robot's alliance so that {@link #showAlliance()} and the post-alliance-shift
   * restore use the correct color. Call this once after the Driver Station connection is
   * established (e.g. in {@code teleopInit}).
   *
   * @param isRed {@code true} for red alliance, {@code false} for blue.
   */
  public void setAlliance(boolean isRed) {
    isRedAlliance = isRed;
  }

  // ---- Command factories -----------------------------------------------------

  /**
   * Returns a command that <em>continuously</em> polls a shot-likelihood value and maps it to
   * a Blinkin solid color:
   *
   * <ul>
   *   <li>0.0 → red (no chance)
   *   <li>~0.5 → yellow/orange (even odds)
   *   <li>1.0 → green (near-certain)
   * </ul>
   *
   * <p>The mapping is a simple linear interpolation through the Blinkin's solid-color set-points,
   * so the color transitions smoothly as the likelihood changes. All likelihood calculation is
   * handled outside this subsystem; the LED code only renders what it receives.
   *
   * @param likelihoodSupplier provides the current shot probability in [0.0, 1.0]
   */
  public Command showShotLikelihood(DoubleSupplier likelihoodSupplier) {
    return run(
        () -> {
          shotLikelihood = Math.max(0.0, Math.min(1.0, likelihoodSupplier.getAsDouble()));
          currentMode = LEDMode.SHOT_LIKELIHOOD;
        });
  }

  /**
   * Returns a command that flashes the strip between red and blue while it runs, then reverts to
   * the robot's alliance color when it ends. Bind this to a trigger that is active for the
   * duration of the alliance-shift game period, e.g.:
   *
   * <pre>{@code
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
   * Returns a command that switches to a solid alliance color (red or blue). Runs once; the
   * color holds until another command changes the mode.
   */
  public Command showAlliance() {
    return runOnce(() -> currentMode = LEDMode.ALLIANCE);
  }

  /** Returns a command that sets the Blinkin to its "black" / off set-point. Runs once. */
  public Command off() {
    return runOnce(() -> currentMode = LEDMode.OFF);
  }

  // ---- Private helpers -------------------------------------------------------

  /**
   * Linearly interpolates a Blinkin PWM set-point from {@link #LIKELIHOOD_LOW_PWM} (red) to
   * {@link #LIKELIHOOD_HIGH_PWM} (green). The Blinkin's solid-color block is monotone between
   * those values, so intermediate PWM values pass through orange, gold, and yellow automatically.
   */
  private double likelihoodToPWM(double likelihood) {
    return LIKELIHOOD_LOW_PWM + likelihood * (LIKELIHOOD_HIGH_PWM - LIKELIHOOD_LOW_PWM);
  }

  /** Alternates the Blinkin between solid red and solid blue every half-cycle. */
  private void applyAllianceShiftAnimation() {
    boolean showRed = (int) (animationTimer.get() / ALLIANCE_SHIFT_FLASH_PERIOD_S) % 2 == 0;
    blinkin.set(showRed ? COLOR_RED : COLOR_BLUE);
  }
}
