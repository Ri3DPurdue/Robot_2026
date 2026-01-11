package frc.robot;

public enum IDs {
    // CAN IDs

    // IDs 0-9 Reserved for drivetrain
    TBD(62, ""),
    
    CLIMBER_MAIN(40, "canivore1");

    public final int id;
    public final String bus;

    private IDs(int id, String bus) {
        this.id = id;
        this.bus = bus;
    }
}
