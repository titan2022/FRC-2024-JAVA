package frc.robot.utility.localization;

import edu.wpi.first.math.geometry.Translation2d;

import java.lang.String;



public enum Note implements FieldObject {

    /* B = Blue side
     * M = Middle
     * R = Red Side
     * 
     * Numbering starts from    
     * 
     */

    NoteB1("B2", new Translation2d( 2.895600003, 1.210056001)),
    NoteB2("B2", new Translation2d( 2.895600003, 2.657856003)),
    NoteB3("B3", new Translation2d( 2.895600003, 4.105656004)),

    NoteM1("M1", new Translation2d( 8.244840008, 0.7528560008)),
    NoteM2("M2", new Translation2d( 8.244840008, 2.429256002)),
    NoteM3("M3", new Translation2d( 8.244840008, 4.105656004)),
    NoteM4("M4", new Translation2d( 8.244840008, 5.782056006)),
    NoteM5("M5", new Translation2d( 8.244840008, 7.458456008)),

    NoteR1("R1", new Translation2d( 8.244840008, 2.657856003)),
    NoteR2("R2", new Translation2d( 8.244840008, 2.657856003)),
    NoteR3("R3", new Translation2d( 8.244840008, 2.657856003));

    private String name = null;
    private Translation2d position = null;

    private Note (String name, Translation2d position){
        this.name = name;
        this.position = position;
    }

    @Override
    public FieldObjectType getType() {
        return FieldObjectType.Note;
    };

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void setName(String name) {
        this.name = name;
    }

    @Override
    public Translation2d getPosition() {
        return position;
    }

    @Override
    public void setPosition(Translation2d position) {
        this.position = position;
    }
}
