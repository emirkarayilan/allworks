package datastructures;

public class CircularNode<T> {
    public T data;              
    public CircularNode<T> next; 

    public CircularNode(T data) {
        this.data = data;      
        this.next = this;     
    }
}
                    