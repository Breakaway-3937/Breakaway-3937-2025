package frc.robot;

//FIXME: Would be stored in folder with Elevator subsystem.
public enum ElevatorPositions {
    L1(0.0),
    L2(0.0),
    L3(0.0),
    L4(0.0),
    CORAL_STATION(0.0);

    private final double position;

    private ElevatorPositions(double position) {
        this.position = position;
    }

    public double getPosition() {
        return position;
    }
}