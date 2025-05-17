package manager;

import maze.MazeTile;
import java.util.Random;

public class MazeManager {
    private MazeTile[][] grid;
    private int width, height;

    public MazeManager(int width, int height) {
        this.width = width;
        this.height = height;
        this.grid = new MazeTile[width][height];
    }

    public void rotateCorridor(int rowIndex) {
        if (rowIndex < 0 || rowIndex >= height) {
            System.out.println("Invalid row index for rotation!");
            return;
        }
    
        MazeTile last = grid[width - 1][rowIndex]; 
    
        for (int x = width - 1; x > 0; x--) {
            grid[x][rowIndex] = grid[x - 1][rowIndex];
        }
    
        grid[0][rowIndex] = last; 
            
        System.out.println("Row " + rowIndex + " rotated!");
    }
    

    public void generateMaze() {
        Random rand = new Random();

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                char type = 'E';  
                int randNum = rand.nextInt(100);

                if (randNum < 10) {
                    type = 'W';  
                } else if (randNum < 20) {
                    type = 'T';  
                } else if (randNum < 30) {
                    type = 'P';
                }

                grid[x][y] = new MazeTile(x, y, type);
            }
        }

        int goalX, goalY;
        do { 
            goalX = rand.nextInt(width);
            goalY = rand.nextInt(height);
        } while (grid[goalX][goalY].getType() != 'E');

        grid[goalX][goalY].setType('G');
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public void placeAgent(agent.Agent agent) {
        grid[agent.currentX][agent.currentY].hasAgent = true;
    }

    public void updateAgentLocation(agent.Agent agent, int oldX, int oldY) {
        grid[oldX][oldY].hasAgent = false;
        grid[agent.currentX][agent.currentY].hasAgent = true;
    }

    public boolean isValidMove(int x, int y, String direction) {
        int newX = x, newY = y;
        switch (direction) {
            case "UP":    newY = y - 1; break;
            case "DOWN":  newY = y + 1; break;
            case "LEFT":  newX = x - 1; break;
            case "RIGHT": newX = x + 1; break;
            default:      return false;
        }
        if (newX < 0 || newX >= width || newY < 0 || newY >= height) {
            return false;
        }
        return grid[newX][newY].isTraversable();
    }

    public MazeTile getTile(int x, int y) {
        return grid[x][y];
    }

    public void printMazeSnapshot() {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                System.out.print(grid[x][y] + " ");
            }
            System.out.println();
        }
        System.out.println();
    }
}
