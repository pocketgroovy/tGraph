package roadgraph;

import java.util.Comparator;

public class NodeComparator implements Comparator<Node> {

	/* (non-Javadoc)
	 *  comparing each distance
	 *  to be used for PriorityQueue
	 * @see java.util.Comparator#compare(java.lang.Object, java.lang.Object)
	 */
	@Override
	public int compare(Node o1, Node o2) {
		// TODO Auto-generated method stub
		if(o1.getDistance() < o2.getDistance()){
			return -1;
		}
		else if(o1.getDistance() > o2.getDistance()){
			return 1;
		}
		return 0;
	}

}
