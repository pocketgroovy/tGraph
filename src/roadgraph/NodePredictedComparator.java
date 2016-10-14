package roadgraph;

import java.util.Comparator;

public class NodePredictedComparator implements Comparator<Node> {

	/* (non-Javadoc)
	 * 	comparing each predicted distance
	 *  to be used for PriorityQueue for A* search
	 * @see java.util.Comparator#compare(java.lang.Object, java.lang.Object)
	 */
	@Override
	public int compare(Node o1, Node o2) {
		// TODO Auto-generated method stub
		if(o1.getPredictedDistance() < o2.getPredictedDistance()){
			return -1;
		}
		else if(o1.getPredictedDistance() > o2.getPredictedDistance()){
			return 1;
		}
		return 0;
	}

}
