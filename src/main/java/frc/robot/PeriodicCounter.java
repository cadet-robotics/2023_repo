package frc.robot;

/**
 * Counts periodic calls to only do an action every X cycles of periodic
 * @author MattaRama
 */
public class PeriodicCounter {
    private int periods = 0;
    private int cycleCount;

    /**
     * @param cycleCount The amount of periodic cycles before the event is called
     */
    public PeriodicCounter(int cycleCount) {
        this.cycleCount = cycleCount;
    }

    // increments periodic counter
    public void periodic(Runnable function) {
        periods++;

        if (periods >= cycleCount) {
            periods = 0;
            function.run();
        }
    }
}
