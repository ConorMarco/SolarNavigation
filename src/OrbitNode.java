import java.util.ArrayList;
import java.util.List;

public class OrbitNode extends Node {
	CelestialBody body;
	double radius;
	PhysicsEngine physics;
	private ArrayList<NeighborCost> neighbors;
	
	private static final double TIME_COST = 1;
	private static final double DV_COST = 1e3;	
	
	public OrbitNode(CelestialBody body, double radius, PhysicsEngine physics) {
		this.body = body;
		this.radius = radius;
		this.physics = physics;
		
		neighbors = new ArrayList<NeighborCost>();
	}

	@Override
	public ArrayList<NeighborCost> neighbors(double time) {
		return neighbors;
	}
	
	public void addNeighbor(OrbitNode n) {
		double cost;
		if(this.body == n.body) {
			cost = hohmannDV(body, this.radius, n.radius) * DV_COST
					+ hohmannTime(body, this.radius, n.radius) * TIME_COST;
		} else if(this.body.getParent() == n.body) {
			cost = toParentOrbit(this.body, this.radius, n.radius) * DV_COST
					+ toParentEllipseTime(this.body, n.radius) * TIME_COST;
		} else if(this.body == n.body.getParent()) {
			cost = toChildDV(this.body, this.radius, n.body, n.radius) * DV_COST
					+ toChildTime(this.body, this.radius, n.body, n.radius) * TIME_COST;
		} else {
			return;
		}
		
		neighbors.add(new NeighborCost(n, cost, 0));
	}

	@Override
	public double getHeuristic(Node goal, double time) {
		double w1 = physics.getGravityWell(body, radius);
		double w2 = physics.getGravityWell(((OrbitNode)goal).body, ((OrbitNode)goal).radius);
		return Math.abs(w1 - w2);
	}
	
	private static double hohmannDV(CelestialBody body, double r1, double r2) {
		return hohmannHalf(body, r1, r2, 1) + hohmannHalf(body, r2, r1, -1);
	}
	
	private static double hohmannTime(CelestialBody body, double r1, double r2)  {
		double mu = body.getMass() * PhysicsEngine.G;
		double r_sum = r1 + r2;
		return Math.PI * Math.sqrt((r_sum*r_sum*r_sum)/(8*mu));
	}
	
	//flag is 1 for entering, -1 for exiting. r1, r2 if entering; r2, r1 if exiting
	private static double hohmannHalf(CelestialBody body, double r1, double r2, int flag) {
		double mu = body.getMass() * PhysicsEngine.G;
		double v = Math.sqrt(2*r2/(r1+r2)) - 1;
		return flag * v * Math.sqrt(mu / r1);
	}
	
	private static double toParentOrbit(CelestialBody body, double r, double parent_r) {
		return toParentEllipse(body, r, parent_r) + hohmannHalf(body.getParent(), parent_r, body.getOrbit(), -1);
	}
	
	private static double toParentEllipse(CelestialBody body, double r, double parent_r) {
		double extra = hohmannHalf(body.getParent(), body.getOrbit(), parent_r, 1);
		//TODO use vis viva equation to calculate extra more accurately
		
		return hohmannHalf(body, r, 1e8, 1) + extra;
	}
	
	private static double toParentEllipseTime(CelestialBody body, double parent_r) {
		return hohmannTime(body.getParent(), body.getOrbit(), parent_r);
	}
	
	//assuming alignment
	private static double toPeerCost(CelestialBody b1, double r1, CelestialBody b2, double r2) {
		if(b1.getParent() != b2.getParent()) {
			return 0;
		}
		
		return toParentEllipse(b1, r1, b2.getOrbit()) + hohmannHalf(b2, 1e8, r2, -1);
	}
	
	private static double toChildDV(CelestialBody p, double r_p, CelestialBody c, double r_c) {
		return Math.max(hohmannDV(p, r_p, c.getOrbit()), hohmannDV(p, r_p, c.getOrbit() * (1 - 4.0/5)));
	}
	
	public List<TransferInstruction> navigateTo(OrbitNode n) {
		ArrayList<TransferInstruction> transfers = new ArrayList<TransferInstruction>();
		
		if(n.body == this.body) {
			double r1 = this.radius;
			double r2 = n.radius;
			int dir = r1 < r2 ? 1 : -1;
			
			double dv1 = hohmannHalf(this.body, r1, r2, 1);
			transfers.add(new TransferInstruction(dv1, dir));
			double dv2 = hohmannHalf(this.body, r2, r1, -1);
			transfers.add(new TransferInstruction(dv2, dir, r2, this.body));
		} else if(n.body == this.body.getParent()) {
			int dir = this.body.getOrbit() < n.radius ? 1 : -1;
			transfers.add(new TransferInstruction(toParentEllipse(this.body, this.radius, n.radius), dir));
			transfers.add(new TransferInstruction(hohmannHalf(n.body, n.radius, this.body.getOrbit(), -1), dir, n.radius, n.body));
		} else if(n.body.getParent() == this.body) {
			double child_r = n.body.getOrbit();
			int dir = this.radius < child_r ? 1 : -1;
			double ship_r = this.radius;
			if(Math.abs(child_r - this.radius)/this.radius < 1/8) {
				//transfer from r_p to 4*r2/5
				double r1 = this.radius;
				double r2 = n.body.getOrbit() * (1 - 4.0*dir/5);
				ship_r = r2;
				
				double dv1 = hohmannHalf(this.body, r1, r2, 1);
				transfers.add(new TransferInstruction(dv1, -dir));
				double dv2 = hohmannHalf(this.body, r2, r1, -1);
				transfers.add(new TransferInstruction(dv2, -dir, r2, this.body));
			}
			
			double transferTime = hohmannTime(this.body, ship_r, child_r);
			double avAngleChange = (Math.sqrt(PhysicsEngine.G * body.getMass() / (ship_r*ship_r*ship_r))
					+ Math.sqrt(PhysicsEngine.G * body.getMass() / (child_r*child_r*child_r))) / 2;
			double desiredAngle = avAngleChange * transferTime;
			transfers.add(new TransferInstruction(desiredAngle, n.body, dir));
			//wait for alignment
					
			//transfer to child orbit
			transfers.add(new TransferInstruction(hohmannHalf(this.body, ship_r, child_r, 1), dir));
			transfers.add(new TransferInstruction(hohmannHalf(this.body, child_r, ship_r, -1), dir, child_r, this.body));
					
			//chase to account for error
			transfers.add(new TransferInstruction(n.body));
			
			//move into correct child orbit
			double ship_r2 = n.body.SOI();
			transfers.add(new TransferInstruction(hohmannHalf(n.body, ship_r2 , n.radius, 1), -1));
			transfers.add(new TransferInstruction(hohmannHalf(n.body, n.radius, ship_r2, -1), -1, n.radius, n.body));
		}
		
		return transfers;
	}
	
	private static double toChildTime(CelestialBody p, double r_p, CelestialBody c, double r_c) {
		//return 1/2 orbital period on avergae for alignment (on avergae, try to determine analytically later)
		return c.getOrbitalPeriod()/2;
	}

}
