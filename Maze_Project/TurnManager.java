package manager;

import java.util.ArrayList;
import java.util.List;

import agent.Agent;
import datastructures.Queue;

public class TurnManager {
    private Queue<Agent> agentQueue = new Queue<>();
    private int currentRound;

    public TurnManager() {
        this.agentQueue = new Queue<>();
        this.currentRound = 1;
    }

    public void advanceTurn() {
        Agent currentAgent = agentQueue.dequeue();  
        agentQueue.enqueue(currentAgent);  
        currentRound++; 
    }

    public List<Agent> getAgentList() {
        List<Agent> agents = new ArrayList<>();
        Queue<Agent> tempQueue = new Queue<>();
    
        while (!agentQueue.isEmpty()) {
            Agent agent = agentQueue.dequeue();
            agents.add(agent);
            tempQueue.enqueue(agent);
        }
    
        while (!tempQueue.isEmpty()) {
            agentQueue.enqueue(tempQueue.dequeue());
        }
    
        return agents;
    }
    
    
    public void addAgent(agent.Agent agent) {
        agentQueue.enqueue(agent);
    }

    public Agent getCurrentAgent() {
        return agentQueue.peek(); 
    }

    public boolean allAgentsFinished() {
        return agentQueue.isEmpty();
    }

    public void printTurnOrder() {
        System.out.print("Current Turn Order: ");
        
        Queue<Agent> tempQueue = new Queue<>();
        while (!agentQueue.isEmpty()) {
            Agent agent = agentQueue.dequeue();
            System.out.print("Agent " + agent.id + " -> ");
            tempQueue.enqueue(agent);
        }
        
        while (!tempQueue.isEmpty()) {
            agentQueue.enqueue(tempQueue.dequeue());
        }
        
        System.out.println();
    }

    public void logTurnSummary(Agent agent) {
        System.out.println("Turn " + currentRound + ": Agent " + agent.id + " moved.");
    }
}
