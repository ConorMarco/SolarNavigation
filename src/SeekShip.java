import java.util.ArrayList;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class SeekShip extends Ship {
	ArrayList<Vector3D> seekpoints;
	Vector3D current;
	int index = 0;
	double lookaheadSq;
	
	public SeekShip(double mass, Vector3D startPos, Vector3D startVel, double thrust, ArrayList<Vector3D> seekpoints, double lookahead) {
		super(mass, startPos, startVel, thrust);
		this.seekpoints = seekpoints;
		this.lookaheadSq = lookahead * lookahead;
		current = seekpoints.get(0);
	}
	
	@Override
	public Vector3D getSteering(float dt, double time) {
		while(this.getPos().distanceSq(current) < lookaheadSq) {
			if(++index < seekpoints.size()) {
				current = seekpoints.get(index);
			} else {
				return Vector3D.ZERO;
			}
		}
		
		return current.subtract(getPos()).normalize().scalarMultiply(thrust);
	}

}
