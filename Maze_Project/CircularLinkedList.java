package datastructures;

public class CircularLinkedList<T> {
    private CircularNode<T> head; 
    private int size;             

    public CircularLinkedList() {
        this.head = null;
        this.size = 0;
    }

    public boolean isEmpty() {
        return head == null;
    }

    public void add(T item) {
        CircularNode<T> newNode = new CircularNode<>(item);
        if (isEmpty()) {
            head = newNode;       
        } else {
            CircularNode<T> tail = head;
            for (int i = 1; i < size; i++) {
                tail = tail.next; 
            }
            tail.next = newNode;  
            newNode.next = head;  
        }
        size++;                   
    }


    public T remove() {
        if (isEmpty()) {
            System.out.println("List is empty! Cannot remove.");
            return null;
        }
        T data = head.data;       
        if (size == 1) {
            head = null;         
        } else {
            
            CircularNode<T> tail = head;
            for (int i = 1; i < size; i++) {
                tail = tail.next;
            }
            head = head.next;    
            tail.next = head;     
        }
        size--;                   
        return data;
    }

    public void rotate(int k) {
        if (isEmpty() || k <= 0) return;
        int steps = k % size;     
        for (int i = 0; i < steps; i++) {
            head = head.next;     
        }
    }

    public void printList() {
        if (isEmpty()) {
            System.out.println("Circular list is empty.");
            return;
        }
        CircularNode<T> current = head;
        System.out.print("CircularList: ");
        for (int i = 0; i < size; i++) {
            System.out.print(current.data + " -> ");
            current = current.next;
        }
        System.out.println("(back to head)");
    }

    public int size() {
        return size;
    }
}
