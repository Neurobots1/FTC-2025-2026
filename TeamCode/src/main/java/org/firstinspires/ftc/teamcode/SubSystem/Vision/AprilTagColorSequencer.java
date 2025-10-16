package org.firstinspires.ftc.teamcode.SubSystem.Vision;

public class AprilTagColorSequencer {

    //AprilTagColorSequencer sequencer = new AprilTagColorSequencer();

    //int detectedId = 21;
    //String sequence = sequencer.getColorSequence(detectedId);
    //System.out.println("Color sequence for tag " + detectedId + ": " + sequence);
    public String getColorSequence(int tagId) {
        switch (tagId) {
            case 20:
                return "blue";
            case 21:
                return "green purple purple";  // gpp
            case 22:
                return "purple green purple";  // pgp
            case 23:
                return "purple purple green";  // ppg
            case 34:
                return "red";
            default:
                return "Unknown";
        }
    }
}