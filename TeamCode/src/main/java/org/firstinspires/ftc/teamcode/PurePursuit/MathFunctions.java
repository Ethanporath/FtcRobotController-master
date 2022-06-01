package org.firstinspires.ftc.teamcode.PurePursuit;


import static java.lang.Math.*;


import java.util.ArrayList;

public class MathFunctions {
    /**
     * Makes sure an angle is within the range -180 to 180 degrees
     *
     * @param angle
     * @return
     */
    public static double AngleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius,
                                                          Point linePoint1, Point linePoint2) {
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }

        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);

        double quadraticA = 1.0 + pow(m1, 2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * pow(m1, 2) * x1);

        double quadraticC = ((pow(m1, 2) * pow(x1, 2))) - (2.0 * y1 * m1 * x1) + pow(y1, 2) - pow(radius, 2);

        ArrayList<Point> allPoint = new ArrayList<>();

        try {
            double xRoot1 = (-quadraticB + sqrt(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);

            double yroot1 = m1 * (xRoot1 - x1) + y1;

            //Put back the offset from earlier
            xRoot1 += circleCenter.x;
            yroot1 += circleCenter.y;

            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;

            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoint.add(new Point(xRoot1, yroot1));
            }

            double xroot2 = (-quadraticB - sqrt(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
            double yroot2 = m1 * (xroot2 - x1) + y1;

            xroot2 += circleCenter.x;
            yroot2 += circleCenter.y;

            if (xroot2 > minX && xroot2 < maxX) {
                allPoint.add(new Point(xroot2, yroot2));
            }

        } catch (Exception e) {

        }
        return allPoint;
    }
}
