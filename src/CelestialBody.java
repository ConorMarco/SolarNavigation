import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class CelestialBody {
	private double radius;
	private double safeRadius;
	protected double mass;
	
	private CelestialBody parent;
	private double startAngle;
	private double angleChange;
	private double altitude;
	private double orbit;
	
	private boolean root;
	Vector3D pos;
	
	public CelestialBody(double mass, double radius, CelestialBody parent, double startAngle, double altitude, double orbit) {
		this(mass, radius, null);
		
		root = false;
		this.parent = parent;
		this.startAngle = startAngle;
		angleChange = Math.sqrt(PhysicsEngine.G * parent.getMass() / (orbit*orbit*orbit));

		this.altitude = altitude;
		this.orbit = orbit;
	}
	
	public CelestialBody(double mass, double radius, Vector3D startPos) {
		this.mass = mass;
		this.radius = radius;
		safeRadius = radius;	//Meh, should probably fix this
		parent = null;
		
		root = true;
		pos = startPos;
	}
	
	public double getRadius() {
		return radius;
	}
	
	public double getSafeRadius() {
		return safeRadius;
	}
	
	public double getMass() {
		return mass;
	}
	
	public Vector3D getPos(double time) {
		if(root) {
			return pos;
		} else {
			double angle = startAngle + time * angleChange;
			Vector3D relativePos = new Vector3D(orbit * Math.sin(angle), orbit * Math.cos(angle), 0);
			//TODO handle altitude
			return relativePos.add(parent.getPos(time));
		}
	}
	
	public boolean isRoot() {
		return root;
	}

	public CelestialBody getParent() {
		return parent;
	}

	public double getOrbit() {
		return orbit;
	}
	
	public double getOrbitalPeriod() {
		return Math.PI / angleChange;
	}

	public double SOI() {
		if(root) { return Double.POSITIVE_INFINITY; }
		return orbit * Math.pow(mass / parent.mass, 0.4);
	}
}
