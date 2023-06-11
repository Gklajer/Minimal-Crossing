package gklajer;

import java.util.Objects;

import org.json.JSONObject;

class Node {
    private int id;
    private Position position;

    public double displaceX = 0;
    public double displaceY = 0;

    public Node(int id, Position position) {
        this.id = id;
        this.position = position;
    }

    public Node(JSONObject node) {
        id = node.getInt("id");
        position = new Position(node.getInt("x"), node.getInt("y"));
    }

    public int getId() {
        return id;
    }

    public Position getPosition() {
        return position;
    }

    public int getX() {
        return position.getX();
    }

    public int getY() {
        return position.getY();
    }

    public void setPosition(Position position) {
        this.position = position;
    }

    public void displace() {
        setPosition(new Position((int) (getX() + displaceX), (int) (getY() + displaceY)));
        displaceX = displaceY = 0;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }

        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }

        Node other = (Node) obj;
        return id == other.id && Objects.equals(position, other.position);
    }

    @Override
    public int hashCode() {
        return Objects.hash(id, position);
    }
}