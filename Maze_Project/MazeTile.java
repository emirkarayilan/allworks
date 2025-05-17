package maze;

public class MazeTile {
    public int x, y;
    public char type;
    public boolean hasAgent;

    public MazeTile(int x, int y, char type) {
        this.x = x;
        this.y = y;
        this.type = type;
        this.hasAgent = false;
    }

    public boolean isTraversable() {
        return type != 'W'; 
    }

    public void setType(char type) {
        this.type = type;
    }

    public char getType() {
        return this.type;
    }

    @Override
    public String toString() {
        if (hasAgent) {
            return "A";
        }
        return String.valueOf(type);  
    }
}
