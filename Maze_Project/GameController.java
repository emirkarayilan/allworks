package main;

import agent.Agent;
import manager.MazeManager;
import manager.TurnManager;
import maze.MazeTile;

import java.util.Random;

public class GameController {
    private MazeManager mazeManager;
    private TurnManager turnManager;
    private int maxTurns, turnCount;

    public GameController(int w, int h, int numAgents, int maxTurns) {
        this.mazeManager  = new MazeManager(w, h);
        this.turnManager  = new TurnManager();
        this.maxTurns     = maxTurns;
        this.turnCount    = 0;
        initializeGame(numAgents);
    }

    private void initializeGame(int numAgents) {
        mazeManager.generateMaze();
        Random rand = new Random();
        for (int i = 0; i < numAgents; i++) {
            int sx = rand.nextInt(mazeManager.getWidth());
            int sy = rand.nextInt(mazeManager.getHeight());
            Agent agent = new Agent(i + 1, sx, sy);
            mazeManager.placeAgent(agent);
            turnManager.addAgent(agent);
        }
    }

    public void runSimulation() {
        Random rand = new Random();
    
        while (turnCount < maxTurns && !turnManager.allAgentsFinished()) {
            System.out.println("Turn " + (turnCount + 1) + ":");
            turnManager.printTurnOrder();
    
            Agent a = turnManager.getCurrentAgent();
            processAgentAction(a);
            turnManager.advanceTurn();
            mazeManager.printMazeSnapshot();
    
        
            System.out.println("--- Agent Move Histories ---");
            for (Agent agent : turnManager.getAgentList()) {
                System.out.print("Agent " + agent.getId() + " Last 5 Moves: ");
                agent.printLastFiveMoves();
            }
            System.out.println("-----------------------------");
    
            turnCount++;
    
            if (turnCount % 5 == 0) {
                int randomRow = rand.nextInt(mazeManager.getHeight());
                mazeManager.rotateCorridor(randomRow);
                System.out.println("[Rotation] Row" + randomRow + " rotated!");
            }
        }
        System.out.println("=== Simulation Complete ===");
        System.out.println("Total Turns: " + turnCount);
    }

    private void processAgentAction(Agent agent) {
        int oldX = agent.currentX, oldY = agent.currentY;
        String[] dirs = {"UP", "DOWN", "LEFT", "RIGHT"};
        String dir = dirs[new Random().nextInt(4)];
    
        if (mazeManager.isValidMove(oldX, oldY, dir)) {
            agent.move(dir);
            MazeTile tile = mazeManager.getTile(agent.currentX, agent.currentY);
    
            switch (tile.type) {
                case 'T': 
                    System.out.println("Agent " + agent.id + " triggered a trap!");
                    agent.backtrack(); 
                    break;
                case 'P': 
                    System.out.println("Agent " + agent.id + " found a power-up!");
                    agent.hasPowerUp = true; 
                    break;
                case 'G': 
                    System.out.println("Agent " + agent.id + " reached the goal!");
                    agent.hasReachedGoal = true; 
                    break;
            }
            
            if (agent.hasPowerUp) {
                agent.usePowerUp();
            }
    
            mazeManager.updateAgentLocation(agent, oldX, oldY);
        } else {
            System.out.println("Agent " + agent.id + " attempted invalid move.");
        }
    }
    
}
