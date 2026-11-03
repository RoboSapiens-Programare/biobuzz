package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;

/**
 * Nominal field positions of the four BIOBUZZ HIVE cell openings, in Pedro Pathing coordinates.
 *
 * <p>Pedro frame: origin (0,0) at the audience-left (red) field corner, +x to the right (toward
 * blue), +y away from the audience (toward the far wall). The field spans [0, 144] on both axes.
 * Heading is in radians, counter-clockwise from +x, with 0 facing +x; it is unused when these poses
 * are used as aiming targets because Tracker only reads x/y.
 *
 * <p>These are absolute field coordinates. The localizer must be seeded in the same frame (for
 * example {@code follower.setPose(startPose)} matching where the robot is actually placed). If the
 * robot is instead zeroed at boot with {@code Pose.zero()}, these values will not line up.
 *
 * <p><b>TODO(verify on field):</b> these numbers are unverified. The questionable value is the
 * cell mouth's offset from the HIVE center along y. This file currently uses half the 30.88 in
 * cell-center spacing (15.44 in), which puts the mouths at y = 56.56 / 87.44. A manual-derived
 * estimate of the mouth center's horizontal projection is ~24.11 in, which would put them at
 * y = 47.89 / 96.11. Check BIOBUZZ manual Figures 9-8, 9-9, 9-11 and 10-3 (or measure the real
 * field), then update CELL_OFFSET / AUDIENCE_Y / FAR_Y. Also verify which cell of each HIVE is up
 * in the starting configuration.
 *
 * <p>Only one cell per HIVE faces up at a time. Up and down cells share the same x/y; the 2D Pose
 * does not model the height difference (up cell opening roughly 53.5-65.6 in, down cell roughly
 * 25.5 in).
 */
public final class HivePoses {
    private HivePoses() {}

    /** Pedro field length and width, in inches. */
    public static final double FIELD_SIZE = 144.0;

    /** Field center, in inches, in Pedro coordinates. */
    public static final double FIELD_CENTER = FIELD_SIZE / 2.0;

    /** Distance between the red and blue HIVE centers. */
    public static final double HIVE_CENTER_SPACING = 25.5;

    // TODO: confirm this is the mouth-center spacing, not just the beam attach spacing.
    /** Distance between the two cell centers of a single HIVE (12.04 cell depth + 18.84 gap). */
    public static final double CELL_CENTER_SPACING = 30.88;

    public static final double HIVE_OFFSET = HIVE_CENTER_SPACING / 2.0;
    public static final double CELL_OFFSET = CELL_CENTER_SPACING / 2.0;

    /** X of the red HIVE (red side is -x from center). */
    public static final double RED_X = FIELD_CENTER - HIVE_OFFSET;

    /** X of the blue HIVE (blue side is +x from center). */
    public static final double BLUE_X = FIELD_CENTER + HIVE_OFFSET;

    /** Y of the audience-side cell. */
    public static final double AUDIENCE_Y = FIELD_CENTER - CELL_OFFSET;

    /** Y of the far-side cell. */
    public static final double FAR_Y = FIELD_CENTER + CELL_OFFSET;

    public static final Pose RED_AUDIENCE = new Pose(RED_X, AUDIENCE_Y, 0);
    public static final Pose RED_FAR = new Pose(RED_X, FAR_Y, 0);
    public static final Pose BLUE_AUDIENCE = new Pose(BLUE_X, AUDIENCE_Y, 0);
    public static final Pose BLUE_FAR = new Pose(BLUE_X, FAR_Y, 0);

    public static final Pose[] HIVE_CELLS = {RED_AUDIENCE, RED_FAR, BLUE_AUDIENCE, BLUE_FAR};
}
