package edu.ahs.robotics.util;

import org.junit.Test;

import edu.ahs.robotics.util.ftc.RingBuffer;

import static org.junit.Assert.*;

public class RingBufferTest {

    @Test
    public void testRingBuffer(){
        int length = 10;

        RingBuffer<Integer> ringBuffer = new RingBuffer<>(length,0);

        int index = 0; //index for both loops. crappy hack that keeps track of total progress

        for (int i = 0; i < length; i++) {
            int val = ringBuffer.insert(index);
            assertEquals(0,val);
            index++;
        }
        for (int i = 0; i < length; i++) {
            int val = ringBuffer.insert(index);
            assertEquals(index - 10,val);
            index ++;
        }
    }

    @Test
    public void testRingBufferWith1(){
        int length = 1;

        RingBuffer<Integer> ringBuffer = new RingBuffer<>(length,0);

        int lastVal = -1;

        for (int i = 1; i < 10; i++) {
            int val = ringBuffer.insert(i);
            assertEquals(lastVal + 1, val);
            lastVal = val;
        }
    }
}