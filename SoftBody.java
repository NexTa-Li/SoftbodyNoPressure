import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;

import geometry.Point2D;
import geometry.Rectangle;
import geometry.Vector2D;

//Non pressurized softbody
public class SoftBody {
    List<SoftBody> softBodies;
    List<MassPoint> points;
    List<Integer> externalPoints;
    List<Spring> springs;

    public final double SPRING_CONSTANT;
    public final double SPRING_DAMPING;
    public final double MASS;

    int[] edgePointIndices;
    Point2D closestPoint = new Point2D();

    Rectangle boundingBox;

    double xMin, xMax, yMin, yMax;

    public boolean keyUp, keyDown, keyLeft, keyRight;

    public SoftBody(double x, double y, int width, int height, double ks, double kd, int restLength, double mass,
            List<SoftBody> softBodies) {
        this.points = new ArrayList<>();
        this.externalPoints = new ArrayList<>();
        this.springs = new ArrayList<>();
        this.SPRING_CONSTANT = ks;
        this.SPRING_DAMPING = kd;
        this.MASS = mass;

        this.softBodies = softBodies;

        this.boundingBox = new Rectangle(x, y, 1, 1);
        this.edgePointIndices = new int[3];
        createPointsAndSprings(x, y, width, height, restLength);
    }

    /**
     * Creates springs between the points
     * 
     * Each point should have a spring connecting to the point below, to the right,
     * diagonally below and to the right, and diagonally below and to the left.
     * 
     * Unique cases:
     * - Points on the left edge should not have a diagonal spring to the left
     * - Points on the right edge should not have a spring to the right
     * - Points on the bottom edge should only have a spring to the right
     * 
     * @param i
     * @param j
     */
    void createPointsAndSprings(double x, double y, int width, int height, int restLength) {
        int rows = height / restLength;
        int cols = width / restLength;

        double diagonalRestLength = Math.sqrt(2 * (restLength * restLength));

        int[][] pointIndices = new int[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                points.add(new MassPoint(x + j * restLength, y + i * restLength));
                pointIndices[i][j] = points.size() - 1;

                // springs to the right
                if (j < cols - 1) {
                    springs.add(new Spring((i * cols) + j, (i * cols) + j + 1, restLength));
                }

                // springs below
                if (i < rows - 1) {
                    springs.add(new Spring((i * cols) + j, ((i + 1) * cols) + j, restLength));
                }

                // springs diagonally below and to the right
                if (i < rows - 1 && j < cols - 1) {
                    springs.add(new Spring((i * cols) + j, ((i + 1) * cols) + j + 1, diagonalRestLength));
                }

                // springs diagonally below and to the left
                if (i < rows - 1 && j > 0) {
                    springs.add(new Spring((i * cols) + j, ((i + 1) * cols) + j - 1, diagonalRestLength));
                }
            }
        }

        // Traverse the top edge
        for (int i = 0; i < cols; i++) {
            externalPoints.add(pointIndices[0][i]);
        }

        // Traverse the right edge
        for (int i = 1; i < rows; i++) {
            externalPoints.add(pointIndices[i][cols - 1]);
        }

        // Traverse the bottom edge
        if (rows > 1) { // Check if there is more than one row
            for (int i = cols - 2; i >= 0; i--) {
                externalPoints.add(pointIndices[rows - 1][i]);
            }
        }

        // Traverse the left edge
        if (cols > 1) { // Check if there is more than one column
            for (int i = rows - 2; i > 0; i--) {
                externalPoints.add(pointIndices[i][0]);
            }
        }

        // for (int i = 0; i < pointIndices.length; i++) {
        // for (int j = 0; j < pointIndices[i].length; j++) {
        // System.out.print(pointIndices[i][j] + " ");
        // }
        // System.out.println();
        // }

        // System.out.println("External points: " + externalPoints);
    }

    public void idle() {
        accumulateForces();
        integrateHuen();
    }

    void accumulateForces() {
        accumulateGravityForce();
        accumulateSpringForce();
    }

    void accumulateGravityForce() {

        for (MassPoint massPoint : points) {
            massPoint.setForce(Const.GRAVITY.getX(), Const.GRAVITY.getY());
            massPoint.getForce().multiply(this.MASS); // F = m * a

            if (keyUp) {
                massPoint.addForceY(-Const.USER_FORCE * this.MASS);
            }
            if (keyDown) {
                massPoint.addForceY(Const.USER_FORCE * this.MASS);
            }
            if (keyLeft) {
                massPoint.addForceX(-Const.USER_FORCE * this.MASS);
            }
            if (keyRight) {
                massPoint.addForceX(Const.USER_FORCE * this.MASS);
            }
        }
    }

    void accumulateSpringForce() {
        Vector2D forceVector = new Vector2D(0.0, 0.0);
        int p1, p2;
        double x1, x2, y1, y2, distance, force, velocityX, velocityY;

        for (int i = 0; i < springs.size(); i++) {
            p1 = springs.get(i).getP1();
            p2 = springs.get(i).getP2();

            x1 = points.get(p1).getPositionX();
            x2 = points.get(p2).getPositionX();
            y1 = points.get(p1).getPositionY();
            y2 = points.get(p2).getPositionY();

            // calculate distance, This is current length of spring
            distance = points.get(p1).getPosition().distance(points.get(p2).getPosition());

            // calculate force
            if (distance != 0) { // skip 0 to avoid division by 0
                velocityX = points.get(p1).getVelocityX() - points.get(p2).getVelocityX();
                velocityY = points.get(p1).getVelocityY() - points.get(p2).getVelocityY();

                force = (distance - springs.get(i).getLength()) * SPRING_CONSTANT +
                        (velocityX * (x1 - x2) + velocityY * (y1 - y2)) * SPRING_DAMPING / distance;

                // calculate force vector
                forceVector.setX(force * ((x1 - x2) / distance));
                forceVector.setY(force * ((y1 - y2) / distance));

                // add force to points
                points.get(p1).subtractForce(forceVector);
                points.get(p2).addForce(forceVector);
            }
        }
    }

    void integrateHuen() {

        double deltaRotationY, deltaRotationX;

        ArrayList<Vector2D> ForceSaved = new ArrayList<Vector2D>();
        ArrayList<Vector2D> VelocitySaved = new ArrayList<Vector2D>();

        for (int i = 0; i < points.size(); i++) {
            // handle friction
            applyFriction(i);

            ForceSaved.add(new Vector2D(points.get(i).getForce()));
            VelocitySaved.add(new Vector2D(points.get(i).getVelocity()));

            double dvx = points.get(i).getForceX() / MASS * Const.TIME_STEP;
            double dvy = points.get(i).getForceY() / MASS * Const.TIME_STEP;

            points.get(i).getVelocity().add(dvx, dvy);

            deltaRotationX = points.get(i).getVelocityX() * Const.TIME_STEP;
            deltaRotationY = points.get(i).getVelocityY() * Const.TIME_STEP;

            points.get(i).addPosition(deltaRotationX, deltaRotationY);

            handlePointCollision(i);

            xMin = Math.min(xMin, points.get(i).getPositionX());
            xMax = Math.max(xMax, points.get(i).getPositionX());
            yMin = Math.min(yMin, points.get(i).getPositionY());
            yMax = Math.max(yMax, points.get(i).getPositionY());
        }

        boundingBox.x = xMin;
        boundingBox.y = yMin;
        boundingBox.width = xMax - xMin;
        boundingBox.height = yMax - yMin;

        accumulateForces();

        for (int i = 0; i < points.size(); i++) {

            double dvx = (points.get(i).getForceX() + ForceSaved.get(i).getX()) / MASS * Const.TIME_STEP;
            double dvy = (points.get(i).getForceY() + ForceSaved.get(i).getY()) / MASS * Const.TIME_STEP;

            points.get(i).setVelocityX(VelocitySaved.get(i).getX() + dvx / 2.0);
            points.get(i).setVelocityY(VelocitySaved.get(i).getY() + dvy / 2.0);

            deltaRotationX = points.get(i).getVelocityX() * Const.TIME_STEP;
            deltaRotationY = points.get(i).getVelocityY() * Const.TIME_STEP;

            points.get(i).addPosition(deltaRotationX, deltaRotationY);

            handlePointCollision(i);

            if (externalPoints.contains(i)) {
                handleRigidCollision(i);
                handleSoftBodyCollision(i);
            }
        }
    }

    void applyFriction(int i) {
        if (points.get(i).isCollided()) {
            // scale the velocity down (1.0 represents no friction i.e full velo retention)
            points.get(i).getVelocity().multiply(1.0 - Const.SURFACE_FRICTION_COEFFICIENT);
            points.get(i).setCollided(false);
            return;
        }
    }

    void handleRigidCollision(int i) {
        // collision with ground
        if (points.get(i).getPosition().getY() >= SoftBodyView.PANEL_HEIGHT) {
            points.get(i).getPosition().setY(SoftBodyView.PANEL_HEIGHT);
            points.get(i).getVelocity().setY(0);

            // set points as collided
            points.get(i).setCollided(true);
        }
        // collision with ceiling
        if (points.get(i).getPosition().getY() <= 0) {
            points.get(i).getPosition().setY(0);
            points.get(i).getVelocity().setY(0);

            points.get(i).setCollided(true);
        }

        // collision with walls
        if (points.get(i).getPosition().getX() >= SoftBodyView.PANEL_WIDTH) {
            points.get(i).getPosition().setX(SoftBodyView.PANEL_WIDTH);
            points.get(i).getVelocity().setX(0);

            points.get(i).setCollided(true);
        }

        if (points.get(i).getPosition().getX() < 0) {
            points.get(i).getPosition().setX(0);
            points.get(i).getVelocity().setX(0);

            points.get(i).setCollided(true);
        }
    }

    void handlePointCollision(int i) {
        for (int j = 0; j < points.size(); j++) {

            if (i == j) {
                continue;
            }

            double distance = points.get(i).getPosition().distance(points.get(j).getPosition());

            if (distance >= Const.MASS_POINT_RADIUS || distance == 0) {
                continue;
            }

            // find the vector between the two points
            int p1 = i;
            int p2 = j;

            double x1 = points.get(p1).getPositionX();
            double x2 = points.get(p2).getPositionX();

            double y1 = points.get(p1).getPositionY();
            double y2 = points.get(p2).getPositionY();

            Vector2D collisionVector = new Vector2D(x2 - x1, y2 - y1);
            collisionVector.divide(distance);

            // normalize the vector
            collisionVector.normalize();
            Vector2D normalVector = new Vector2D(collisionVector);

            // take velocities into account
            double p = 2 * (points.get(p1).getVelocityX() * normalVector.getX()
                    + points.get(p1).getVelocityY() * normalVector.getY()
                    - points.get(p2).getVelocityX() * normalVector.getX()
                    + points.get(p2).getVelocityY() * normalVector.getY())
                    / (this.MASS + this.MASS);

            double vx1 = points.get(p1).getVelocityX() - p * this.MASS * normalVector.getX();
            double vy1 = points.get(p1).getVelocityY() - p * this.MASS * normalVector.getY();
            double vx2 = points.get(p2).getVelocityX() + p * this.MASS * normalVector.getX();
            double vy2 = points.get(p2).getVelocityY() + p * this.MASS * normalVector.getY();

            points.get(p1).setVelocity(vx1, vy1);
            points.get(p2).setVelocity(vx2, vy2);
        }
    }

    void handleSoftBodyCollision(int i) {
        for (int j = 0; j < softBodies.size(); j++) {

            // skip self collision
            if (softBodies.get(j) == this) {
                continue;
            }

            // skip if theres no collision
            if (!checkCollision(points.get(i).getPosition(), softBodies.get(j))) {
                continue;
            }
            // System.out.println("Collision detected with: " + j);

            // skip if this point is at the same position as the closest point
            if (points.get(i).getPosition().equals(closestPoint)) {
                continue;
            }

            int p1 = edgePointIndices[0];
            int p2 = edgePointIndices[1];

            // normal Vector
            Vector2D n = new Vector2D(points.get(i).getPositionX() - closestPoint.getX(),
                    points.get(i).getPositionY() - closestPoint.getY());

            // velocity of this point
            double vx = points.get(i).getVelocityX();
            double vy = points.get(i).getVelocityY();

            // other bodies point velocity
            double o_vx = softBodies.get(j).points.get(p1).getVelocityX();
            double o_vy = softBodies.get(j).points.get(p1).getVelocityY();

            n.divide(2.0);

            // Calculate new positions for each point
            double newX1 = closestPoint.getX() + n.getX();
            double newY1 = closestPoint.getY() + n.getY();

            double newX2 = softBodies.get(j).points.get(p1).getPositionX() + n.getX();
            double newY2 = softBodies.get(j).points.get(p1).getPositionY() + n.getY();

            // handle vertex collisions
            if (p2 == -1) {
                // System.out.println("Handling vertex collision with: " + j);

                points.get(i).setPosition(newX1, newY1);
                softBodies.get(j).points.get(p1).setPosition(newX2, newY2);

                n.normalize();
                // calculate impulse
                double p = 2 * ((vx * n.getX() + vy * n.getY()) - (o_vx * n.getX() + o_vy *
                        n.getY()))
                        / (MASS + softBodies.get(j).MASS);

                double vx1 = vx - (p * softBodies.get(j).MASS * n.getX());
                double vy1 = vy - (p * softBodies.get(j).MASS * n.getY());

                double vx2 = o_vx + (p * MASS * n.getX());
                double vy2 = o_vy + (p * MASS * n.getY());

                vx1 *= Const.BOUNCINESS;
                vy1 *= Const.BOUNCINESS;
                vx2 *= Const.BOUNCINESS;
                vy2 *= Const.BOUNCINESS;

                points.get(i).setVelocity(vx1, vy1);
                softBodies.get(j).points.get(p1).setVelocity(vx2, vy2);

                continue; // skip the rest
            }

            double newX3 = softBodies.get(j).points.get(p2).getPositionX() + n.getX();
            double newY3 = softBodies.get(j).points.get(p2).getPositionY() + n.getY();

            // Calculate the new positions for the edge points using the closestToSingle
            // vector

            points.get(i).setPosition(newX1, newY1);

            softBodies.get(j).points.get(p1).setPosition(newX2, newY2);
            softBodies.get(j).points.get(p2).setPosition(newX3, newY3);

            // normalize for the collision calculations
            n.normalize();

            // handle edge collisions
            double edgeVx = (o_vx + softBodies.get(j).points.get(p2).getVelocityX()) / 2.0;
            double edgeVy = (o_vy + softBodies.get(j).points.get(p2).getVelocityY()) / 2.0;

            double p = 2 * ((vx * n.getX() + vy * n.getY()) - (edgeVx * n.getX() + edgeVy * n.getY()))
                    / (MASS + softBodies.get(j).MASS);

            double vx1 = vx - (p * softBodies.get(j).MASS * n.getX());
            double vy1 = vy - (p * softBodies.get(j).MASS * n.getY());

            double vx2 = edgeVx + (p * MASS * n.getX());
            double vy2 = edgeVy + (p * MASS * n.getY());

            vx1 *= Const.BOUNCINESS;
            vy1 *= Const.BOUNCINESS;
            vx2 *= Const.BOUNCINESS;
            vy2 *= Const.BOUNCINESS;

            points.get(i).setVelocity(vx1, vy1);
            softBodies.get(j).points.get(p1).setVelocity(vx2, vy2);
            softBodies.get(j).points.get(p2).setVelocity(vx2, vy2);

        }
    }

    public boolean checkCollision(Point2D position, SoftBody other) {
        // Check if the point is inside the bounding box of the other soft body
        if (!other.boundingBox.contains(position.getX(), position.getY())) {
            return false;
        }

        Line2D ray = new Line2D.Double(position.getX(), 0, position.getX(), position.getY());

        int points = other.externalPoints.size();
        int intersectionCount = 0;

        // set the closest point to the first point in the list
        closestPoint = new Point2D(other.points.get(other.externalPoints.get(0)).getPosition());

        for (int i = 0; i < points; i++) {
            int p1 = other.externalPoints.get(i);
            int p2 = other.externalPoints.get((i + 1) % points);

            Point2D edgeStart = other.points.get(p1).getPosition();
            Point2D edgeEnd = other.points.get(p2).getPosition();

            Point2D tempClosestPoint = SoftBodyUtil.closestPointOnLineSegment(position, edgeStart, edgeEnd);

            double tempDistance = position.distance(tempClosestPoint);
            double oldDistance = position.distance(closestPoint);

            // check if the closest point is on an edge and closer than the previous closest
            if (tempDistance <= oldDistance) {
                edgePointIndices[0] = p1;
                edgePointIndices[1] = tempClosestPoint.equals(edgeStart) ? -1 : p2;
                // this should probably only be set if theres actually a collision
                closestPoint.setLocation(tempClosestPoint.getX(), tempClosestPoint.getY());
            }

            /*
             * check if the ray intersects the edge.
             * Also check if the ray is at the same height as a point on
             * the edge, if it is, then the next edge will be intersected, so we skip this
             */
            if (!SoftBodyUtil.checkIntersection(ray, edgeStart, edgeEnd)) {
                continue;
            }

            if (position.getY() != edgeEnd.getY()) {
                intersectionCount++;
            }
        }

        // If the number of intersections is odd, then the point is inside the soft body
        return intersectionCount % 2 == 1;
    }

    public Rectangle getBoundingBox() {
        return this.boundingBox; // Privacy sacrifice for performance
    }

    public ArrayList<MassPoint> getPoints() {
        return new ArrayList<>(this.points);
    }

    public ArrayList<Integer> getExternalPoints() {
        return new ArrayList<>(this.externalPoints);
    }

    public ArrayList<Spring> getSprings() {
        return new ArrayList<>(this.springs);
    }

    public int[] getXArr() {
        int[] xArr = new int[externalPoints.size()];
        for (int i = 0; i < externalPoints.size(); i++) {
            double x = points.get(externalPoints.get(i)).getPositionX();
            xArr[i] = (int) x;
        }
        return xArr;
    }

    public int[] getYArr() {
        int[] yArr = new int[externalPoints.size()];
        for (int i = 0; i < externalPoints.size(); i++) {
            double y = points.get(externalPoints.get(i)).getPositionY();
            yArr[i] = (int) y;
        }
        return yArr;
    }
}
