import java.util.ArrayList;

public abstract class Node {
	protected double cost = Float.POSITIVE_INFINITY;
	protected Node parent;
	protected ArrayList<Node> neighbors;
	
	public abstract ArrayList<NeighborCost> neighbors(double time);
	protected double time;
	
	public boolean updateCost(double newCost, Node node, double newTime, Node goal) {
		if(newCost < cost && newCost + getHeuristic(goal, newTime) < cost + getHeuristic(goal, time) ) {
			cost = newCost;
			parent = node;
			time = newTime;
			return true;
		} else {
			return false;
		}
	}
	
	public double getCost() {
		return cost;
	}
	
	public double getPriority(Node goal) {
		return getCost() + getHeuristic(goal, time);
	}
	
	public void reset(Node goal) {
		cost = Float.POSITIVE_INFINITY;
		parent = null;
	}
	
	public Node getParent() {
		return parent;
	}
	
	public abstract double getHeuristic(Node goal, double time);
}
