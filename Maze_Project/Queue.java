package datastructures;

public class Queue<T> {
    private static class QueueNode<T> {
        T data;
        QueueNode<T> next;

        public QueueNode(T data) {
            this.data = data;
            this.next = null;
        }
    }

    private QueueNode<T> front; 
    private QueueNode<T> rear;  

    public Queue() {
        this.front = null; 
        this.rear = null;
    }

    public void enqueue(T item) {
        QueueNode<T> newNode = new QueueNode<>(item); 
        if (isEmpty()) {
            front = rear = newNode; 
        } else {
            rear.next = newNode; 
            rear = newNode;      
        }
    }

    public T dequeue() {
        if (isEmpty()) {
            System.out.println("Queue is empty! Cannot dequeue.");
            return null;
        }
        T data = front.data;     
        front = front.next;      
        if (front == null) {   
            rear = null;
        }
        return data;             
    }

    public T peek() {
        if (isEmpty()) {
            System.out.println("Queue is empty! Cannot peek.");
            return null;
        }
        return front.data;
    }

    public boolean isEmpty() {
        return front == null; 
    }

    public void printQueue() {
        QueueNode<T> current = front;
        System.out.println("Queue (front -> rear):");
        while (current != null) {
            System.out.println(current.data);
            current = current.next;
        }
    }
}