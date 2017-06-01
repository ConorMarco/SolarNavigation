import java.util.ArrayList;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class NaiveNode extends Node {
	protected ArrayList<NeighborCost> neighbors;
	protected Vector3D pos;
	protected PhysicsEngine physics;
	
	public NaiveNode(Vector3D pos, PhysicsEngine physics) {
		this.pos = pos;
		neighbors = new ArrayList<NeighborCost>();
		this.physics = physics;
	}
	
	public Vector3D getPos() {
		return pos;
	}
	
	public void addNeighbor(NaiveNode n) {
		double d = pos.distance(n.getPos());
		neighbors.add(new NeighborCost(n, d, d));
	}

	@Override
	public ArrayList<NeighborCost> neighbors(double time) {
		return neighbors;
	}

	@Override
	public double getHeuristic(Node goal, double time) {
		return pos.distance(((NaiveNode)goal).pos) + Math.abs(
				physics.getGravityWell(pos, time) - physics.getGravityWell(((NaiveNode)goal).getPos(), time));
	}

}
