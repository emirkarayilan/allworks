package main;

public class Main {
    public static void main(String[] args) {
        GameController game = new GameController(6, 6, 2, 50); // 6 ya 6 lık labirent, 2 ajan, 50 tur 
        game.runSimulation();
    }
}
