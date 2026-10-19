package frc.robot.commands;

/**
 * A stretch of a BLine path: opens when a BLine event fires and closes once the path has handed
 * off past {@code segments} translation targets (1 = the end of the segment the event sits on).
 * Fed BLine's logged FollowPath/translationElementIndex; later fires of the same event are ignored.
 */
public class BLineEventWindow {
    private final int segments;
    private double index = Double.NaN;
    private int handoffsSinceEvent = 0;
    private boolean opened = false;

    /** Open from the event to the end of its own segment. */
    public BLineEventWindow() {
        this(1);
    }

    /** Open from the event across {@code segments} translation targets (team 2026-10-14: path 1 = 2). */
    public BLineEventWindow(int segments) {
        this.segments = segments;
    }

    /** Close and forget: call at the start and end of every run. */
    public void reset() {
        opened = false;
        handoffsSinceEvent = 0;
    }

    /** BLine's current translation target index (from the double logging consumer). */
    public void onIndex(double translationElementIndex) {
        if (opened && translationElementIndex != index) {
            handoffsSinceEvent++;
        }
        index = translationElementIndex;
    }

    /** BLine event fired. Only the first fire per run opens the window. */
    public void onEvent() {
        if (!opened) {
            opened = true;
            handoffsSinceEvent = 0;
        }
    }

    /** True from the event until BLine has handed off past {@code segments} translation targets. */
    public boolean isOpen() {
        return opened && handoffsSinceEvent < segments;
    }
}
