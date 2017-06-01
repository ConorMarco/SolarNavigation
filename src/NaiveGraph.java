import java.util.ArrayList;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class NaiveGraph extends Graph {
	public static final int GRID_SPACING = 10000000;

	public NaiveGraph(PhysicsEngine physics) {
		int min_i = (int)(physics.minX()/GRID_SPACING);
		int min_j = (int)(physics.minY()/GRID_SPACING);
		int min_k = (int)(physics.minZ()/GRID_SPACING);
		int max_i = (int)(physics.maxX()/GRID_SPACING);
		int max_j = (int)(physics.maxY()/GRID_SPACING);
		int max_k = (int)(physics.maxZ()/GRID_SPACING);
		
		NaiveNode[][][] array = new NaiveNode[max_i-min_i][max_j-min_j][max_k-min_k];
		
		for(int i=min_i; i<max_i; ++i) {
			for(int j=min_j; j<max_j; ++j) {
				for(int k=min_k; k<max_k; ++k) {
					Vector3D pos = new Vector3D(i*GRID_SPACING, j*GRID_SPACING, k*GRID_SPACING);
					if(!physics.insideBody(pos))
						array[i-min_i][j-min_j][k-min_k] = new NaiveNode(pos, physics);
				}
			}
		}
		
		nodes = new ArrayList<Node>();
		for(int i=0; i<array.length; ++i) {
			for(int j=0; j<array[0].length; ++j) {
				for(int k=0; k<array[0][0].length; ++k) {
					if(array[i][j][k] != null) {
						nodes.add(array[i][j][k]);
						addNeighbors(array, i, j, k);
					}
				}
			}
		}
		
	}

	private void addNeighbors(NaiveNode[][][] array, int i, int j, int k) {
		addNeighbor(array[i][j][k], array, i-1, j, k);
		addNeighbor(array[i][j][k], array, i+1, j, k);
		addNeighbor(array[i][j][k], array, i, j-1, k);
		addNeighbor(array[i][j][k], array, i, j+1, k);
		addNeighbor(array[i][j][k], array, i, j, k-1);
		addNeighbor(array[i][j][k], array, i, j, k+1);
	}

	private void addNeighbor(NaiveNode node, NaiveNode[][][] array, int i, int j, int k) {
		if(i >= 0 && i < array.length) {
			if(j >= 0 && j < array[0].length) {
				if(k >= 0 && k < array[0][0].length && array[i][j][k] != null) {
					node.addNeighbor(array[i][j][k]);
				}
			}
		}
		
	}
	
	public Node findCloseNode(Vector3D pos) {
		for(Node node : nodes) {
			if(((NaiveNode)node).getPos().distanceSq(pos) < GRID_SPACING * GRID_SPACING) {
				return node;
			}
		}
		return nodes.get(0);
	}

}
