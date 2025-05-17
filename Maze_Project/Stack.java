package datastructures;

public class Stack<T> {
    private StackNode<T> top;

    public Stack() {
        this.top = null;
    }

    public void push(T item) {
        StackNode<T> newNode = new StackNode<>(item);
        newNode.next = top;
        top = newNode;
    }

    public T pop() {
        if (isEmpty()) {
            System.out.println("Stack is empty! Cannot pop.");
            return null;
        }
        T data = top.data;
        top = top.next;
        return data;
    }

    public T peek() {
        if (isEmpty()) {
            System.out.println("Stack is empty! Cannot peek.");
            return null;
        }
        return top.data;
    }


    public boolean isEmpty() {
        return top == null;
    }

    public void printStack() {
        StackNode<T> current = top;
        System.out.println("Stack (top -> bottom):");
        while (current != null) {
            System.out.println(current.data);
            current = current.next;
        }
    }
}
