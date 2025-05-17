package agent;

import datastructures.Stack;

public class Agent {
    
    private Stack<String> moveHistory;

    public int id;
    public int currentX;
    public int currentY;
    public boolean hasReachedGoal;
    public int totalMoves;
    public int backtracks;
    public boolean hasPowerUp;

    public Agent(int id, int startX, int startY) {
        this.id = id;
        this.currentX = startX;
        this.currentY = startY;
        this.moveHistory = new Stack<>(); 
        this.hasReachedGoal = false;
        this.totalMoves = 0;
        this.backtracks = 0;
        this.hasPowerUp = false;
        recordMove(startX, startY);       
    }

    public void move(String direction) {
        totalMoves++;
        recordMove(currentX, currentY);

        switch (direction) {
            case "UP":
                currentY -= 1; break;
            case "DOWN":
                currentY += 1; break;
            case "LEFT":
                currentX -= 1; break;
            case "RIGHT":
                currentX += 1; break;
            default:
                System.out.println("Invalid direction: " + direction);
        }
    }

    public void usePowerUp() {
        if(hasPowerUp) {
            System.out.println("Agent " + id + " used a power up!");
            hasPowerUp = false;
            totalMoves--;
        } else {
            System.out.println("Agent "+ id +"has no power-ups.");
        }
    }

    public void backtrack() {
        System.out.println("Agent " + id + " is backtracking...");
        for (int i = 0; i < 2; i++) {
            if (!moveHistory.isEmpty()) {
                moveHistory.pop();
            }
        }
        if (!moveHistory.isEmpty()) {
            String[] pos = moveHistory.peek().split(",");
            currentX = Integer.parseInt(pos[0]);
            currentY = Integer.parseInt(pos[1]);
            System.out.println("Agent " + id + " backtracked to (" + currentX + "," + currentY + ")");
        }
        backtracks++;
    }

    public void applyPowerUp() {
        if (!hasPowerUp) {
            System.out.println("Agent " + id + " has no power-up.");
            return;
        }
        System.out.println("Agent " + id + " uses power-up to skip a turn!");
        hasPowerUp = false; 
    }
    

    public void recordMove(int x, int y) {
        moveHistory.push(x + "," + y);
    }

    public int getId() {
        return id;
    }
    
    public void printLastFiveMoves() {
        int count = 0;
        Stack<String> tempStack = new Stack<>();
    
        Stack<String> tempOriginal = new Stack<>();
        
        while (!moveHistory.isEmpty()) {
            String move = moveHistory.pop();
            tempOriginal.push(move);
        }
        while (!tempOriginal.isEmpty()) {
            String move = tempOriginal.pop();
            moveHistory.push(move); 
            if (count < 5) {
                tempStack.push(move);
                count++;
            }
        }
    
        while (!tempStack.isEmpty()) {
            System.out.print(tempStack.pop() + " ");
        }
        System.out.println();
    }


    public String getMoveHistoryAsString() {
        
        Stack<String> temp = new Stack<>();
        StringBuilder sb = new StringBuilder();


        while (!moveHistory.isEmpty()) {
            String pos = moveHistory.pop();
            temp.push(pos);
            sb.insert(0, pos + " ");  
        }

        while (!temp.isEmpty()) {
            moveHistory.push(temp.pop());
        }
        return sb.toString().trim();
    }
}
