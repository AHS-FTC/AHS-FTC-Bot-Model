package edu.ahs.robotics.util.ftc;

import java.util.ArrayList;
import java.util.List;

/**
 * Maintains a wraparound list of entries fixed to a size specified in the constructor.
 * Useful for smoothing out random error, particularly in the domain of time.
 * Takes class T of which array is filled.
 * @author Alex Appleby
 */
public class RingBuffer<T> {
    private List<T> buffer;
    private int index = 0;

    /**
     * @param length Number of entries in the buffer.
     * @param fillElement Default value for elements of the buffer before adding. Usually 0.
     */
    public RingBuffer(int length, T fillElement) {
        buffer = new ArrayList<>();
        for (int i = 0; i < length; i++) {
            buffer.add(fillElement);
        }
    }

    /**
     * Inserts a new element into the buffer, replacing the oldest element which is returned.
     * If the buffer is not yet full, returns 0.
     * @return The oldest element in the buffer.
     */
    public T insert(T t){
        T old = buffer.get(index);
        buffer.set(index,t);

        if(++index >= buffer.size()){ // index wraps
            index = 0;
        }

        return old;
    }

    public List<T> getBuffer (){
        return buffer;
    }
}
