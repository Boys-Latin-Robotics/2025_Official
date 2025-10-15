package frc.robot.support;

/**
 * A triple record maintaining values suitable for instantiation of a
 * {@link edu.wpi.first.math.controller.PIDController} Note that we're using this custom Record rather that depending on
 * com.pathplanner.lib.config.PICConstants to avoid that dependency throughout our config system
 *
 * @param p
 *            Factor for "proportional" control
 * @param i
 *            Factor for "integral" control
 * @param d
 *            Factor for "derivative" control
 */
public record PIDSettings(double p, double i, double d) {}
