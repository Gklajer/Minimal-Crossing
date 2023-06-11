package gklajer;

import org.json.JSONObject;

class Edge {
    private int sourceId;
    private int targetId;

    public Edge(int sourceId, int targetId) {
        this.sourceId = sourceId;
        this.targetId = targetId;
    }

    public Edge(JSONObject edge) {
        this.sourceId = edge.getInt("source");
        this.targetId = edge.getInt("target");
    }

    public int getSourceId() {
        return sourceId;
    }

    public int getTargetId() {
        return targetId;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }

        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }

        Edge other = (Edge) obj;
        return sourceId == other.sourceId && targetId == other.targetId;
    }

    @Override
    public int hashCode() {
        int result = Integer.hashCode(sourceId);
        result = 31 * result + Integer.hashCode(targetId);
        return result;
    }

}