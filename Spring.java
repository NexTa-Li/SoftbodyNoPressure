
/**
 * Spring class
 * 
 * <p>
 * this class stores the indices of its mass points instead of the mass points
 * to prevent too much memory usage.
 * 
 * if you want to take hit on used memory, its possible to add some optimization
 * by storing and updating a line segment between the mass points.
 * </p>
 */
public class Spring {
    int p1, p2; // point indices
    double length; // resting length

    public Spring(int p1, int p2, double length) {
        this.p1 = p1;
        this.p2 = p2;
        this.length = length;
    }

    public int getP1() {
        return p1;
    }

    public int getP2() {
        return p2;
    }

    public double getLength() {
        return this.length;
    }

}
