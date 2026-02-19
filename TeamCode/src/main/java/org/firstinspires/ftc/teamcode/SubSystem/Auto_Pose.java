package org.firstinspires.ftc.teamcode.SubSystem;

import com.pedropathing.geometry.Pose;
import java.io.*;

public class Auto_Pose {

    private static final String PATH = "/sdcard/Pedro/pose.txt";

    public static void save(Pose p) {
        if (p == null) return;

        try {
            File dir = new File("/sdcard/Pedro");
            if (!dir.exists()) dir.mkdirs();

            FileWriter w = new FileWriter(PATH);
            w.write(p.getX() + " " + p.getY() + " " + p.getHeading());
            w.close();
        } catch (Exception ignored) {}
    }

    public static Pose load() {
        try {
            BufferedReader r = new BufferedReader(new FileReader(PATH));
            String[] s = r.readLine().split(" ");
            r.close();

            return new Pose(
                    Double.parseDouble(s[0]),
                    Double.parseDouble(s[1]),
                    Double.parseDouble(s[2])
            );
        } catch (Exception e) {
            return null;
        }
    }
}
