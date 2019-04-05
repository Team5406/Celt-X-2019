package frc.team3256.warriorlib.auto.purepursuit;

import frc.team3256.warriorlib.math.Vector;

import java.util.ArrayList;
import java.util.Arrays;

public class PathGenerator {
    private double spacing;
    private double a = 0, b = 0, tolerance = 0;
    private ArrayList<Vector> points = new ArrayList<>();
    private double maxVel, maxAccel, maxVelk, endVelocity;
    private boolean forward;

    public PathGenerator(double spacing, boolean forward) {
        this.spacing = spacing;
        this.forward = forward;
    }

    public void setSmoothingParameters(double a, double b, double tolerance) {
        this.a = a;
        this.b = b;
        this.tolerance = tolerance;
    }

    public void setVelocities(double maxVel, double maxAccel, double maxVelk, double endVelocity) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxVelk = maxVelk;
        this.endVelocity = endVelocity;
    }

    public void addPoint(Vector point) {
        points.add(point);
    }

    public void addPoints(Vector... points) {
        this.points.addAll(Arrays.asList(points));
    }

    public Path generatePath() {
        Path path = new Path(spacing, forward);
        for (int i = 0; i < points.size() - 1; ++i)
            path.addSegment(points.get(i), points.get(i + 1));
        path.addLastPoint();
        if (tolerance != 0)
            path.smooth(a, b, tolerance);
        path.initializePath(maxVel, maxAccel, maxVelk, endVelocity);
        return path;
    }
}
