import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Main {
	public static final Vector3D START = new Vector3D(1.5e8, 0, 0);
	public static final Vector3D START_VEL = new Vector3D(0, 30, 0);
	public static final Vector3D END = new Vector3D(1e9, 1e9, 0);

	public static void main(String[] args) {
		ArrayList<CelestialBody> solarSystem = new ArrayList<CelestialBody>();
		CelestialBody sun = new CelestialBody(1.989e30, 6.957e5, Vector3D.ZERO);
		solarSystem.add(sun);
		
		ArrayList<Vector3D> direct = new ArrayList<Vector3D>();
		direct.add(END);
		SeekShip ship1 = new SeekShip(1e5, START, START_VEL, 1e-1, direct, 1e3);
		
		PhysicsEngine physics = new PhysicsEngine(sun, solarSystem, ship1);
		
		
		physics.addBody(3.285e23, 2440, sun, 5.791e7);	// Mercury
		physics.addBody(4.867e24, 6052, sun, 1.082e8);	// Venus
		
		CelestialBody earth = physics.addBody(5.972e24, 6371, sun, 1.496e8);	// Earth
		physics.addBody(7.348e22, 1737, earth, 3.844e5);	// Moon
		
		CelestialBody mars = physics.addBody(6.39e23, 3390, sun, 2.279e8);	// Mars
		
		CelestialBody jupiter = physics.addBody(1.898e27, 7.149e4, sun, 7.785e8);	// Jupiter
		physics.addBody(8.932e22, 1821, jupiter, 421.8e3);	// Io
		physics.addBody(4.8e22, 1561, jupiter, 671.1e3);	// Europa
		physics.addBody(1.482e23, 2631, jupiter, 1070e3);	// Ganymede
		physics.addBody(1.076e23, 2410, jupiter, 1883e3);	// Callisto
		
		CelestialBody saturn = physics.addBody(5.683e26, 6.027e4, sun, 1.429e9);	// Saturn
		physics.addBody(1.345e23, 2575, saturn, 1.222e6);	// Titan
		
		physics.addBody(8.681e25, 2.556e4, sun, 2.871e9);	// Uranus
		physics.addBody(1.024e26, 2.476e4, sun, 4.498e9);	// Neptune
		
		//dwarf planets ftw
		physics.addBody(8.958e20, 470, sun, 4.14e8);	// Ceres
		physics.addBody(1.309e22, 1187, sun, 5.9e9);	// Pluto
		
		//Test 1
		/*NaiveGraph ng = new NaiveGraph(physics);
		ArrayList<Node> path = ng.aStar(ng.findCloseNode(START), ng.findCloseNode(END));
		ArrayList<Vector3D> seeks = new ArrayList<Vector3D>(path.size());
		for(Node n : path) {
			seeks.add(((NaiveNode)n).getPos());
		}
		
		SeekShip ship2 = new SeekShip(1e5, START, START_VEL, 1e-1, seeks, 1e3);
		physics.switchShip(ship2);//*/
		
		//Test 2
		/*Note: this test works very well sometimes, but other times the granularity of the physics
		 * engine causes problems. Set simulation rate to 1e-1 or 1e-2 for more stable results, but this
		 * will be very slow. A small amount of error correction on the part of the ship could probably account for this,
		 * but I have not yet implemented this.
		 */
		/*Vector3D earthStart = earth.getPos(0).add(Vector3D.PLUS_I.scalarMultiply(1e4));
		double speed = Math.sqrt(physics.G * earth.getMass() / 1e4);
		Vector3D earthStartVel = Vector3D.PLUS_J.scalarMultiply(speed);
		OrbitGraph og = new OrbitGraph(physics);
		System.out.println("graph built");
		CelestialBody startBody = physics.findBodyOfInfluence(earthStart, 0);
		OrbitNode startNode = og.findClosest(startBody, earthStart, 0);
		List<TransferInstruction> moves = og.getInitTransfers(startNode, earthStart, 0);
		System.out.println("point 2");
		
		ArrayList<Node> path = og.aStar(startNode, og.getOrbit(mars, 0));
		System.out.println("path built");
		moves.addAll(og.getTransfers(path));
		System.out.println("transfers found");
		OrbitShip ship3 = new OrbitShip(1e5, earthStart, earthStartVel, 1e5, moves);
		physics.switchShip(ship3);	
		System.out.println("starting sim");//*/
		
		
		physics.simulate(10, 10000000);
		
		
	}

}
