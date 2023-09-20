
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JPanel;

public class SoftBodyView extends JPanel implements Runnable, KeyListener {

    Thread thread;

    static final int PANEL_WIDTH = 1200;// 1620
    static final int PANEL_HEIGHT = 700; // 900
    static final int REFRESH_RATE = 240;

    List<MassPoint> points;
    List<Spring> springs;

    List<SoftBody> bodies;

    public SoftBodyView() {
        this.setPreferredSize(new Dimension(PANEL_WIDTH, PANEL_HEIGHT));
        this.setBackground(Color.black);
        this.setFocusable(true);
        this.requestFocus();
        this.addKeyListener(this);

        createBodies();

        this.thread = new Thread(this);
        this.thread.start();

        JFrame frame = new JFrame("Soft Body");
        frame.add(this);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }

    void createBodies() {
        this.bodies = new ArrayList<SoftBody>();

        SoftBody body = new SoftBody(25, 360, 500, 300, 1500, 25, 40, 0.75, bodies);
        bodies.add(body);
        int y = 25;
        int x = 200;
        for (int j = 0; j < 2; j++) {
            for (int i = 0; i < 1; i++) {

                SoftBody b = new SoftBody(x * i + 10, y, 200, 140, 2000, 25, 20, 0.75, bodies);
                bodies.add(b);

            }
            y += 200;
        }

    }

    @Override
    public void run() {

        double drawInterval = 1000000000 / REFRESH_RATE;
        double delta = 0;
        long lastTime = System.nanoTime();
        long currentTime;

        while (thread != null) {
            currentTime = System.nanoTime();

            delta += (currentTime - lastTime) / drawInterval;

            lastTime = currentTime;

            if (delta >= 1) {
                for (int i = 0; i < bodies.size(); i++)
                    bodies.get(i).idle();

                revalidate();
                repaint();
                delta--;
            }
        }
    }

    @Override
    protected void paintComponent(Graphics graphics) {
        super.paintComponent(graphics);
        // graphics 2d
        Graphics2D g = (Graphics2D) graphics;
        g.setStroke(new BasicStroke(2));

        for (int i = 0; i < bodies.size(); i++) {
            SoftBody body = bodies.get(i);

            points = body.getPoints();
            List<Integer> externalPoints = body.getExternalPoints();
            springs = body.getSprings();

            g.setColor(Color.gray);

            g.fillPolygon(body.getXArr(), body.getYArr(), body.getXArr().length);
            for (int j = 0; j < springs.size(); j++) {

                g.drawLine((int) points.get(springs.get(j).getP1()).getPositionX(),
                        (int) points.get(springs.get(j).getP1()).getPositionY(),
                        (int) points.get(springs.get(j).getP2()).getPositionX(),
                        (int) points.get(springs.get(j).getP2()).getPositionY());
            }

            final double POINT_SIZE = 2;

            for (int j = 0; j < externalPoints.size(); j++) {

                g.setColor(Color.lightGray);
                g.drawLine((int) points.get(externalPoints.get(j)).getPositionX(),
                        (int) points.get(externalPoints.get(j)).getPositionY(),
                        (int) points.get(externalPoints.get((j + 1) % externalPoints.size())).getPositionX(),
                        (int) points.get(externalPoints.get((j + 1) % externalPoints.size())).getPositionY());
                g.setColor(Color.red);
                g.fillOval((int) (points.get(externalPoints.get(j)).getPositionX() - POINT_SIZE),
                        (int) (points.get(externalPoints.get(j)).getPositionY() - POINT_SIZE), (int) (2 * POINT_SIZE),
                        (int) (2 * POINT_SIZE));

            }
        }
    }

    @Override
    public void keyPressed(KeyEvent e) {

        if (e.getKeyCode() == KeyEvent.VK_UP)
            bodies.get(0).keyUp = true;
        if (e.getKeyCode() == KeyEvent.VK_DOWN)
            bodies.get(0).keyDown = true;
        if (e.getKeyCode() == KeyEvent.VK_LEFT)
            bodies.get(0).keyLeft = true;
        if (e.getKeyCode() == KeyEvent.VK_RIGHT)
            bodies.get(0).keyRight = true;
        if (e.getKeyCode() == KeyEvent.VK_SPACE)
            bodies.get(0).points.get(0).isFixed = !bodies.get(0).points.get(0).isFixed;
    }

    @Override
    public void keyReleased(KeyEvent e) {

        if (e.getKeyCode() == KeyEvent.VK_UP)
            bodies.get(0).keyUp = false;
        if (e.getKeyCode() == KeyEvent.VK_DOWN)
            bodies.get(0).keyDown = false;
        if (e.getKeyCode() == KeyEvent.VK_LEFT)
            bodies.get(0).keyLeft = false;
        if (e.getKeyCode() == KeyEvent.VK_RIGHT)
            bodies.get(0).keyRight = false;

    }

    public static void main(String[] args) {
        new SoftBodyView();
    }

    @Override
    public void keyTyped(KeyEvent e) {
        // TODO Auto-generated method stub

    }
}