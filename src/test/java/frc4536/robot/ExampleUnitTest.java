package frc4536.robot;

import org.junit.*;

import frc4536.lib.VirtualMotor;
 
import static org.junit.Assert.*;

public class ExampleUnitTest{

    @Test
    public void testVirtualMotor(){
        VirtualMotor m_motor = new VirtualMotor(6);
        m_motor.set(1);
        assertEquals(1, m_motor.get(),0.01);
    }
}
