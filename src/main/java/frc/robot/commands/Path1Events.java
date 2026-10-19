package frc.robot.commands;

/**
 * Path 1 auto's BLine event flags (team 2026-10-16). Each event key flips a flag and RobotContainer
 * binds the mechanisms to the flags. BLine's keys are global, so events only count while the path 1
 * auto runs -- running one part on its own must never leave a shot or the intake on -- and every
 * flag clears when the auto starts or stops.
 */
public class Path1Events {
    private boolean running, intake, stow, feedShot, hubShot;
    private boolean feedShotOn = true; // false = "feed_shot" ignored for this run
    // 3-step path 1 only (team 2026-10-17): the first hub shot starts at once -- flywheels + hood pre-spun
    // from "intake_stop", and no start-of-shot reverse. firstShot clears when that shot ends.
    private boolean fastFirstShot, firstShot, preSpin;

    /** The auto starts: flags cleared, events count from now. */
    public void start() {
        stop();
        running = true;
    }

    /**
     * The 3-step path 1 autos start (team 2026-10-17): "feed_shot" ignored, and the first hub shot is
     * fast (pre-spin from "intake_stop", no start-of-shot reverse).
     */
    public void startThreeStep() {
        stop();
        feedShotOn = false;
        fastFirstShot = firstShot = true;
        running = true;
    }

    /** The auto ends (or is cancelled): events stop counting, every flag cleared. */
    public void stop() {
        running = intake = stow = feedShot = hubShot = false;
        fastFirstShot = firstShot = preSpin = false;
        feedShotOn = true;
    }

    /** "intake": LT held. */
    public void onIntake() { if (running) intake = true; }

    /** "intake_stop": LT let go. */
    public void onIntakeStop() {
        if (running) {
            intake = false;
            preSpin = fastFirstShot && firstShot && !hubShot;
        }
    }

    /** "stow": intake and hood in (stays set until the auto ends; the binding acts on the rising edge). */
    public void onStow() { if (running) stow = true; }

    /** "feed_shot": LB's feed shot + feed aim. */
    public void onFeedShot() { if (running && feedShotOn) feedShot = true; }

    /** "feed_stop": feed shot + feed aim off. */
    public void onFeedStop() { if (running) feedShot = false; }

    /** "hub_shot": RT's hub aim + shot. LT lets go here (team: part 2's intake runs until this). */
    public void onHubShot() {
        if (running) {
            hubShot = true;
            intake = false;
            preSpin = false; // the shot takes the launcher over at speed
        }
    }

    /** The hold after part 1 ends its hub shot. */
    public void endHubShot() {
        hubShot = false;
        firstShot = false;
    }

    public boolean intake() { return intake; }
    public boolean stow() { return stow; }
    public boolean feedShot() { return feedShot; }
    public boolean hubShot() { return hubShot; }
    /** Flywheels + hood spinning up for the first hub shot (3-step only), nothing fed. */
    public boolean preSpin() { return preSpin; }
    /** True while the 3-step auto's first hub shot runs: that shot skips the start-of-shot reverse. */
    public boolean skipShotReverse() { return fastFirstShot && firstShot && hubShot; }
}
