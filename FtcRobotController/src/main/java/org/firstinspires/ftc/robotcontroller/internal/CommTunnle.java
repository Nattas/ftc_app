package org.firstinspires.ftc.robotcontroller.internal;

import java.util.ArrayList;

public class CommTunnle<T>{
    public ArrayList<T> comm=new ArrayList<>();
    public ArrayList<OnTunnle<T>> onTunnle=new ArrayList<>();

    public CommTunnle(){

    }

    public void addReceiver(OnTunnle<T> tunnle){
        onTunnle.add(tunnle);
    }

    public void removeReceiver(OnTunnle<T> tunnle){
        onTunnle.remove(tunnle);
    }

    public void send(T s){
        comm.add(s);
        for(int a=0;a<onTunnle.size();a++){
            onTunnle.get(a).onReceive(s);
        }
    }

    public interface OnTunnle<T>{
        void onReceive(T response);
    }
}
