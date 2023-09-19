import java.util.ArrayList;
import java.util.List;

import javax.swing.Spring;

//Non pressurized softbody
public class SoftBody {
    List<MassPoint> points;
    List<Spring> springs;

    final double SPRING_CONSTANT;
    final double SPRING_DAMPING;

    final double MASS;

    public SoftBody(double x, double y, double ks, double kd, double mass) {
        this.points = new ArrayList<>();
        this.springs = new ArrayList<>();
        this.SPRING_CONSTANT = ks;
        this.SPRING_DAMPING = kd;
        this.MASS = mass;
    }
}
