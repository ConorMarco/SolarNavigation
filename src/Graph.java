import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.PriorityQueue;

public abstract class Graph {
	public static final int INIT_QUEUE_SIZE = 1000;
	
	protected ArrayList<Node> nodes;
	PhysicsEngine physics;
	
	public void reset(Node goal) {
		for(Node n : nodes) {
			n.reset(goal);
		}
	}
	
	public ArrayList<Node> aStar(Node start, Node goal) {
		reset(goal);
		
		HashSet<Node> checked = new HashSet<Node>();
		PriorityQueue<Node> unchecked = new PriorityQueue<Node>(INIT_QUEUE_SIZE, new NodeComparator(goal));
		
		start.updateCost(0, null, 0, goal);
		unchecked.add(start);
		
		while(true) {
			Node current = unchecked.poll();
			if(current == goal) {
				return createPath(start, goal, 1);
			}
			checked.add(current);
			
			System.out.println("hdmof");
			
			for(NeighborCost nc : current.neighbors(current.time)) {
				nc.neighbor.updateCost(nc.cost + current.getCost(), current, nc.time, goal);
				if(!checked.contains(nc.neighbor) && !unchecked.contains(nc.neighbor))
					unchecked.add(nc.neighbor);
			}
		}
	}
	
	private ArrayList<Node> createPath(Node start, Node end, int count) {
		ArrayList<Node> result;
		if(end == start) {
			result = new ArrayList<Node>(count);
		} else {
			result = createPath(start, end.getParent(), count+1);
		}
		
		result.add(end);
		return result;
	}
	
	private class NodeComparator implements Comparator<Node> {
		private Node goal;
		
		public NodeComparator(Node goal) {
			this.goal = goal;
		}
		
		@Override
		public int compare(Node a, Node b) {
			if(a.getPriority(goal) < b.getPriority(goal)) {
				return -1;
			} else if(a.getPriority(goal) > b.getPriority(goal)) {
				return 1;
			}
			
			if(a.getCost() > b.getCost()) {
				return -1;
			} else if(a.getCost() < b.getCost()) {
				return 1;
			}
			
			return 0;
		}
		
	}
}
