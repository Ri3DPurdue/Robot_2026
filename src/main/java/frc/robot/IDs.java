package frc.robot;

public enum IDs {
    // CAN IDs

    // IDs 0-9 Reserved for drivetrain
    INTAKE_PIVOT(10, "canivore1"),
    INTAKE_ROLLERS(11, "canivore1"),
    INDEXER_BELT(20, "canivore1"),
    INDEXER_FEEDER(21, "canivore1"),
    SHOOTER_TOP_FLYWHEEL(30, "canivore1"),
    SHOOTER_BOTTOM_FLYWHEEL(31, "canivore1"),
    CLIMBER_MAIN(40, "canivore1");
    
    public final int id;
    public final String bus;

    private IDs(int id, String bus) {
        this.id = id;
        this.bus = bus;
    }
}
