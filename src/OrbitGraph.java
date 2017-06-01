import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class OrbitGraph extends Graph {
	Map<CelestialBody, ArrayList<OrbitNode>> tree;
	PhysicsEngine physics;
	
	public OrbitGraph(PhysicsEngine physics) {
		tree = new HashMap<CelestialBody, ArrayList<OrbitNode>>();
		nodes = new ArrayList<Node>();
		this.physics = physics;
		
		for(CelestialBody body : physics.getBodies()) {
			tree.put(body, new ArrayList<OrbitNode>());
		}
		
		for(CelestialBody body : physics.getBodies()) {
			OrbitNode low = new OrbitNode(body, body.mass * 1e-21, physics);
			OrbitNode med = new OrbitNode(body, body.mass * 3e-21, physics);
			OrbitNode high = new OrbitNode(body, body.mass * 1e-20, physics);
			low.addNeighbor(med); low.addNeighbor(high);
			med.addNeighbor(low); med.addNeighbor(high);
			high.addNeighbor(low); high.addNeighbor(med);
			tree.get(body).add(low); tree.get(body).add(med); tree.get(body).add(high);
			nodes.add(low); nodes.add(med); nodes.add(high);
			
			if(!body.isRoot()) {
				CelestialBody p = body.getParent();
				OrbitNode n = new OrbitNode(p, body.getOrbit(), physics);
				nodes.add(n);
				for(OrbitNode peer : tree.get(p)) {
					peer.addNeighbor(n);
					n.addNeighbor(peer);
					peer.addNeighbor(high);
					high.addNeighbor(peer);
				}
				tree.get(p).add(n);
			}
		}
	}
	
	public OrbitNode findClosest(CelestialBody body, Vector3D pos, double time) {
		double radius = pos.distance(body.getPos(time));
		
		double minChange = Double.MAX_VALUE;
		OrbitNode best = null;
		
		for(OrbitNode n : tree.get(body)) {
			if(Math.abs(n.radius - radius) < minChange) {
				minChange = n.radius - radius;
				best = n;
			}
		}
		
		return best;
	}
	
	public OrbitNode getOrbit(CelestialBody b, int i) {
		return tree.get(b).get(i);
	}
	
	public List<TransferInstruction> getInitTransfers(OrbitNode start, Vector3D pos, double time) {
		OrbitNode temp = new OrbitNode(start.body, start.body.getPos(time).distance(pos), physics);
		return temp.navigateTo(start);
	}
	
	public static List<TransferInstruction> getTransfers(List<Node> path) {
		ArrayList<TransferInstruction> ins = new ArrayList<TransferInstruction>();
		
		for(int i=0; i < path.size()-1; ++i) {
			OrbitNode current = (OrbitNode)path.get(i);
			OrbitNode next = (OrbitNode)path.get(i+1);
			ins.addAll(current.navigateTo(next));
		}
		
		return ins;
	}
}
