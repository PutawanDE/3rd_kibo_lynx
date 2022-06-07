package jp.jaxa.iss.kibo.rpc.defaultapk;

public class Vector3 {
    private double x, y, z;

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double size() {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }

    public double dot(Vector3 v) {
        return x * v.getX() + y * v.getY() + z * v.getZ();
    }

    public Vector3 cross(Vector3 v) {
        double newX = y * v.getZ() - z * v.getY();
        double newY = z * v.getX() - x * v.getZ();
        double newZ = x * v.getY() - y * v.getX();

        return new Vector3(newX, newY, newZ);
    }

    public Vector3 scale(double s) {
        return new Vector3(s*x, s*y, s*z);
    }

    public Vector3 sum(Vector3 v) {
        return new Vector3(x+v.getX(), y+v.getY(), z+v.getZ());
    }
}