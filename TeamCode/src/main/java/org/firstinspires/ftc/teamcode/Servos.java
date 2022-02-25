package org.firstinspires.ftc.teamcode;

public class Servos {

    public enum ServoHeights {

        collectLeft(0.20),
        collectRight(0.825),

        intakeLeft(0.145),
        intakeRight(0.89),

        dumpLeft(0.375),
        dumpRight(0.67),

        dumpLefttop(0.645),
        dumpRighttop(0.36),

        dumpLeftmid(0.775),
        dumpRightmid(0.26),

        dumpLeftbottom(0.875),
        dumpRightbottom(0.16),

        initLeft(0.455),
        initRight(0.71),

        scanLeft(0.605),
        scanRight(0.42);


        public final double value;
        ServoHeights(double value) {
            this.value = value;
        }
    }

    public enum ServoCollect {
        in(-1),
        out(1),
        Hold(0);
        public final double value;

        private ServoCollect(double value) {
            this.value = value;
        }
    }

}
