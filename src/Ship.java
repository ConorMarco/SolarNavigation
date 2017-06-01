import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public abstract class Ship {
	protected double thrust;
	protected double mass;
	private double invMass;
	private double invMass2;
	private Vector3D pos;
	private Vector3D vel;
	
	public Ship(double mass, Vector3D startPos, Vector3D startVel, double thrust) {
		this.mass = mass;
		invMass = 1/mass;
		invMass2 = invMass/2;
		vel = startVel;
		pos = startPos;
		this.thrust = thrust;
	}
	
	public abstract Vector3D getSteering(float dt, double time);
	
	public double getMass() {
		return mass;
	}

	public Vector3D getPos() {
		return pos;
	}
	
	public Vector3D getVel() {
		return vel;
	}
	
	public void applyForce(float dt, Vector3D force) {
		pos = pos.add(dt, vel).add(dt * dt * invMass2, force);
		vel = vel.add(dt * invMass, force);
	}
}
