import java.awt.Graphics;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JPanel;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class PhysicsEngine extends JPanel {
	private static final long serialVersionUID = 1L;

	public static final double G = 6.67e-20; //km, kg, s
	
	CelestialBody root;
	private ArrayList<CelestialBody> bodies;
	private Ship ship;
	
	private double time = 0;
	
	JFrame frame;
	
	
	public PhysicsEngine(CelestialBody root, ArrayList<CelestialBody> list, Ship ship) {
		super();
		this.root = root;
		bodies = list;
		this.ship = ship;
		
		// <drawing>
		frame = new JFrame("BasicJPanel");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setSize(1400,1000);

		// Make the panel object the content pane of the JFrame.
		// This puts it into the drawable area of frame, and now
		// we do all our drawing to panel, using paintComponent(), above.
		frame.setContentPane(this);
		frame.setVisible(true);
		// <\drawing>
	}
	
	public void physicsTick(float dt) {
		Vector3D netForce = Vector3D.ZERO;
		netForce = netForce.add(ship.getSteering(dt, time));
		
		time += dt;
		
		for(CelestialBody body : bodies) {
			netForce = netForce.add(calculateForce(body));
		}
		
		ship.applyForce(dt, netForce);
	}
	
	private Vector3D calculateForce(CelestialBody body) {
		Vector3D bodyPos = body.getPos(time);
		Vector3D difference = bodyPos.subtract(ship.getPos());
		double magnitude = G * ship.getMass() * body.getMass() / Vector3D.distanceSq(ship.getPos(), bodyPos);
		return difference.normalize().scalarMultiply(magnitude);
	}
	
	public void simulate(float interval, int steps) {
		int count = 0;
		for(int i=0; i<steps; i++) {
			physicsTick(interval);
			
			if(++count > 20) {
				frame.repaint();
				count = 0;
			}
		}
	}
	
	
	public CelestialBody addBody(double mass, double radius, CelestialBody parent, double orbit) {
		double angle = Math.random() * Math.PI;
		
		CelestialBody body = new CelestialBody(mass, radius, parent, angle, 0, orbit);
		bodies.add(body);
		return body;
	}
	
	
	//drawing
	 public void paintComponent(Graphics g){
		 int sw = this.getWidth()/2;
		 int sh = this.getHeight()/2;
		 
		 for(CelestialBody body : bodies) {
			 int radius = (int)(body.getRadius()/2e3);
			 g.drawOval((int)(body.getPos(time).getX()/1e7)-radius+sw, (int)(body.getPos(time).getY()/1e7)-radius+sh, 2*radius, 2*radius);
		 }
		 g.drawRect((int)(ship.getPos().getX()/1e7)-5+sw, (int)(ship.getPos().getY()/1e7)-5+sh, 10, 10);
	 }
	 
	 public double getGravityWell(Vector3D pos, double time) {
		 double well = 0;
		 for(CelestialBody body : bodies) {
			 well += G * body.getMass() / pos.distance(body.getPos(time));
		 }
		 return well;
	 }
	 
	 public double getGravityWell(CelestialBody body, double radius) {
		 CelestialBody current = body;
		 double r = radius;
		 
		 double well = 0;
		 
		 while(current != null) {
			 well += G * body.getMass() / r;
			 
			 current = current.getParent();
			 r = body.getOrbit();
		 }
		
		 return well;
	 }
	 
	 public double minX() {return -1e10;}
	 public double maxX() {return 1e10;}
	 public double minY() {return -1e10;}
	 public double maxY() {return 1e10;}
	 public double minZ() {return -1e7;}
	 public double maxZ() {return 1e7;}

	public boolean insideBody(Vector3D pos) {
		for(CelestialBody body : bodies) {
			double r = body.getRadius();
			if(body.getPos(time).distanceSq(pos) < r*r) {
				return true;
			}
		}
		return false;
	}
	
	public void switchShip(Ship newShip) {
		ship = newShip;
	}
	
	public CelestialBody findBodyOfInfluence(Vector3D pos, double time) {
		CelestialBody i = findBodyOfInfluence(pos, time, root);
		if(i == null) {
			return root;
		}
		return i;
	}
	
	private CelestialBody findBodyOfInfluence(Vector3D pos, double time, CelestialBody body) {
		for(CelestialBody b : bodies) {
			if(body == b.getParent()) {
				CelestialBody i = findBodyOfInfluence(pos, time, b);
				if(i != null) {
					return i;
				}
			}
		}
		
		if(pos.distance(body.getPos(time)) < body.SOI()) {
			return body;
		}
		
		return null;
	}

	public List<CelestialBody> getBodies() {
		return bodies;
	}
}
