// File:          MazeRobotController.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.*;
import java.util.*;
// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class MazeRobotController {

  
    // wanted wheel positions
    static double wantedLwheelPos = 0.0;
    static double wantedRwheelPos = 0.0;
    static int wantedDir = 1;
    static double wantedX = 9.5;
    static double wantedY = 9.5;
    static double endOfMazeX = 4.5;
    static double endOfMazeY = -6.5;
    static int shortestPath = Integer.MAX_VALUE;
    static int currentPathCost = 1;
    static LinkedList<Integer> currentPosition;
    static Stack<MazeNode> curLayerNodes = new Stack<>();
    static Stack<MazeNode> nextLayerNodes = new Stack<>();
    static MazeNode curTarget;
    static ArrayList<MazeNode> mazeNodes = new ArrayList<>();
    static int mazeNodeCounter = 1;
    static int[][] maze = new int[22][22];

    // constants
    static final double blockLength = 9.163927052199342;
    static final double rightAngle = 4.403833745517597;
    // This is the main function of your controller.
    // It creates an instance of your Robot instance and
    // it uses its function(s).
    // Note that only one instance of Robot should be created in
    // a controller program.
    // The arguments of the main function can be specified by the
    // "controllerArgs" field of the Robot node
    public static void main(String[] args) {

        // Create the Robot instance.
        Robot robot = new Robot();
        // Get motors
        Motor frMotor = robot.getMotor("front right wheel");
        Motor flMotor = robot.getMotor("front left wheel");
        Motor brMotor = robot.getMotor("back right wheel");
        Motor blMotor = robot.getMotor("back left wheel");
        // Put the motors into arraylists so they are nicer to handle
        ArrayList<Motor> leftMotors = new ArrayList<>(Arrays.asList(flMotor, blMotor));
        ArrayList<Motor> rightMotors = new ArrayList<>(Arrays.asList(frMotor, brMotor));
        ArrayList<ArrayList<Motor>> motors = new ArrayList<>();
        motors.add(leftMotors);
        motors.add(rightMotors);

        // Get distance sensors
        DistanceSensor lDist = robot.getDistanceSensor("so0");
        DistanceSensor fDist = robot.getDistanceSensor("so3");
        DistanceSensor rDist = robot.getDistanceSensor("so7");

        // Get GPS and Compass
        GPS gps = robot.getGPS("gps");
        Compass compass = robot.getCompass("compass");


        // Get pos sensors
        PositionSensor lWheelPos = robot.getPositionSensor("back left wheel sensor");
        PositionSensor rWheelPos = robot.getPositionSensor("back right wheel sensor");

        // Get the time step of the current world.
        int timeStep = (int) Math.round(robot.getBasicTimeStep());

        // Enable devices
        lDist.enable(timeStep);
        fDist.enable(timeStep);
        rDist.enable(timeStep);
        lWheelPos.enable(timeStep);
        rWheelPos.enable(timeStep);
        gps.enable(timeStep);
        compass.enable(timeStep);
        // Create a visualization tool for maze solving
        maze[(int)(wantedX + 10.5)][(int)(wantedY + 10.5)] = mazeNodeCounter;
        // Create root node and its children
        MazeNode start = new MazeNode();
        start.setCost(0);
        mazeNodes.add(start);
        mazeNodes.add(start);
        mazeNodeCounter += 1;
        if (robot.step(timeStep) != -1){
            if (rDist.getValue() < 900){
                start.setrChild(new MazeNode());
            }
            if (fDist.getValue() < 900){
                start.setfChild(new MazeNode());
            }
        }
        // Initialise two stacks for node handling and the direction list
        // Two stacks are needed to optimize path taken by robot
        LinkedList<Integer> dirs = new LinkedList<>();
        
        for (MazeNode mn : start.children()){
            curLayerNodes.add(mn);
        }

        curTarget = curLayerNodes.pop();
        dirs = curTarget.getPath();
        // only called when target found
        currentPosition = curTarget.getPath();

        
        
        // Limit for correcting
        int corrections = 0;
        //Main loop:
        // - perform simulation steps until Webots is stopping the controller
        while (robot.step(timeStep) != -1) {
            // check sensors
            double left = lDist.getValue();
            double front = fDist.getValue();
            double right = rDist.getValue();
            double curLwheelPos = lWheelPos.getValue();
            double curRwheelPos = rWheelPos.getValue();
            double[] direction = compass.getValues();
            double[] position = gps.getValues();
            // if not moving
            if (closeEnough(curLwheelPos, wantedLwheelPos, 0.0001) && closeEnough(curRwheelPos, wantedRwheelPos, 0.0001)){
                // if last turn wasn't accurate enough
                if (closeEnough(java.lang.Math.abs(direction[0]), wantedDir%2, 0.0005) || (corrections > 100)){
                    corrections = 0;
                    // if last straight wasn't accurate enough
                    if (((closeEnough(position[0], wantedX, 0.01) || wantedDir%2 == 0) && (closeEnough(position[2], wantedY, 0.01) || wantedDir%2 == 1)) || (corrections > 100)){
                        corrections = 0;
                        // If robot has no directions to follow, gets new directions, else follows directions
                        
                        if (dirs.peekFirst() == null){
                            if (curTarget != null){
                                if (left > 900 && right > 900 && front > 900){
                                    if(!endOfMaze()){
                                        curTarget.setX(wantedX);
                                        curTarget.setY(wantedY);
                                        mazeNodes.add(curTarget);
                                        maze[(int)(wantedX + 10.5)][(int)(wantedY + 10.5)] = mazeNodeCounter;
                                        mazeNodeCounter += 1;
                                        int totalCost = curTarget.getParent().getCost() + currentPathCost;
                                        curTarget.setCost(totalCost);
                                        currentPathCost = 1;
                                        currentPosition = curTarget.getPath();
                                        curTarget = null;
                                    }
                                }else {
                                    if (left > 900 && right > 900){
                                        if (!endOfMaze()){
                                            currentPathCost += 1;
                                            curTarget.addToPath(0);
                                            dirs.add(0);
                                            maze[(int)(wantedX + 10.5)][(int)(wantedY + 10.5)] = -2;
                                        }
                                    }else if (left > 900 && front > 900){
                                        if (!endOfMaze()){
                                            currentPathCost += 1;
                                            curTarget.addToPath(1);
                                            curTarget.addToPath(0);
                                            dirs.add(1);
                                            dirs.add(0);
                                            maze[(int)(wantedX + 10.5)][(int)(wantedY + 10.5)] = -2;
                                        }
                                    }else if (front > 900 && right > 900){
                                        if (!endOfMaze()){
                                            currentPathCost += 1;
                                            curTarget.addToPath(3);
                                            curTarget.addToPath(0);
                                            dirs.add(3);
                                            dirs.add(0);
                                            maze[(int)(wantedX + 10.5)][(int)(wantedY + 10.5)] = -2;
                                        }
                                    }else {
                                        if (!endOfMaze()){
                                            int totalCost = curTarget.getParent().getCost() + currentPathCost;
                                            curTarget.setCost(totalCost);
                                            currentPathCost = 1;
                                            curTarget.setX(wantedX);
                                            curTarget.setY(wantedY);
                                            
                                            if (curTarget.getCost()+java.lang.Math.abs(curTarget.getX()-endOfMazeX)+java.lang.Math.abs(curTarget.getY()-endOfMazeY) < shortestPath){
                                                if (left < 900) {
                                                    curTarget.setlChild(new MazeNode());
                                                }

                                                if (right < 900){
                                                    curTarget.setrChild(new MazeNode());
                                                }

                                                if (front < 900){
                                                    curTarget.setfChild(new MazeNode());
                                                }

                                                for (MazeNode mn : curTarget.children()){
                                                    if (mn != null){
                                                        nextLayerNodes.add(mn);
                                                    }
                                                }
                                            }
                                            if (maze[(int)(wantedX + 10.5)][(int)(wantedY + 10.5)]==0){
                                                mazeNodes.add(curTarget);
                                                maze[(int)(wantedX + 10.5)][(int)(wantedY + 10.5)] = mazeNodeCounter;
                                                mazeNodeCounter += 1;
                                            } else {
                                                MazeNode mn = mazeNodes.get(maze[(int)(wantedX + 10.5)][(int)(wantedY + 10.5)]);
                                                if (mn.getCost()>curTarget.getCost()){
                                                    mn.kill();
                                                    mazeNodes.add(curTarget);
                                                    maze[(int)(wantedX + 10.5)][(int)(wantedY + 10.5)] = mazeNodeCounter;
                                                    mazeNodeCounter += 1;
                                                } else {
                                                    curTarget.kill();
                                                }
                                            }
                                            currentPosition = curTarget.getPath();
                                            curTarget = null;
                                        }
                                    }
                                    showMaze(maze, mazeNodes);
                                }
                            }else {
                                // if current layer is searched, move on to next layer
                                if (curLayerNodes.empty()){
                                    curLayerNodes = nextLayerNodes;
                                    nextLayerNodes = new Stack<>();
                                    // if all nodes are searched leave main loop
                                    if (curLayerNodes.empty()){
                                        // Enter here exit cleanup code.
                                        showMaze(maze, mazeNodes);
                                        System.out.println("Target: " + endOfMazeX + ", " + endOfMazeY);
                                        System.out.println("Shortest path: " + shortestPath);
                                        dirs = mazeNodes.get(maze[(int)(endOfMazeX+10.5)][(int)(endOfMazeY+10.5)]).getPath();
                                        int dir = 1;
                                        double x = 9.5;
                                        double y = 9.5;
                                        while(dirs.peekFirst()!=null){
                                            int instr = dirs.poll();
                                            if (instr == 1){
                                                dir = (dir +1)%4;
                                            } else if (instr == 3){
                                                dir = (dir +3)%4;
                                            } else {
                                                if (dir == 1){
                                                    x -= 1;
                                                } else if (dir == 3){
                                                    x += 1;
                                                } else if (dir == 2){
                                                    y -= 1;
                                                } else {
                                                    y += 1;
                                                }
                                            }
                                            if (maze[(int)(x+10.5)][(int)(y+10.5)]<0){
                                                maze[(int)(x+10.5)][(int)(y+10.5)] = -400;
                                            }
                                        }
                                        showMaze(maze, mazeNodes);
                                        return;
                                    }
                                }

                                curTarget = curLayerNodes.pop();
                                if (curTarget.isDead()){
                                    curTarget = null;
                                } else {
                                    LinkedList<Integer> pathToNextNode = curTarget.getPath();
                                    // remove same starting path
                                    while(currentPosition.peekFirst() != null && pathToNextNode.peekFirst() != null && currentPosition.peekFirst().equals(pathToNextNode.peekFirst())){
                                        currentPosition.removeFirst();
                                        pathToNextNode.removeFirst();
                                    }
                                    // add current path backwards to directions
                                    for (Integer i : currentPosition){
                                        dirs.addFirst((i+2)%4);
                                    }
                                    // add path to next node to directions
                                    for (Integer i : pathToNextNode){
                                        dirs.add(i);
                                    }
                                }
                            }
                        } else {
                            //testing
                            //end of testing

                            int instruction = dirs.poll();

                            move(instruction, motors);
                        }
                    } else {
                        double x = position[0];
                        double y = position[2];
                        if (closeEnough(position[0], wantedX, 0.02) && closeEnough(position[2], wantedY, 0.02)) {
                            correctPos(motors, x, y, 0.005);
                        } else if (closeEnough(position[0], wantedX, 0.05) && closeEnough(position[2], wantedY, 0.05)) {
                            correctPos(motors, x, y, 0.01);
                        } else if (closeEnough(position[0], wantedX, 0.08) && closeEnough(position[2], wantedY, 0.08)) {
                            correctPos(motors, x, y, 0.02);
                        } else {
                            correctPos(motors, x, y, 0.1);
                        }
                        corrections += 1;
                    }
                } else {
                    double x = direction[0];
                    double y = direction[1];
                    if (closeEnough(java.lang.Math.abs(direction[0]), wantedDir%2, 0.001)) {
                        correctAngle(motors, x, y, 0.001);
                    } else if (closeEnough(java.lang.Math.abs(direction[0]), wantedDir%2, 0.002)) {
                        correctAngle(motors, x, y, 0.002);
                    } else if (closeEnough(java.lang.Math.abs(direction[0]), wantedDir%2, 0.004)) {
                        correctAngle(motors, x, y, 0.005);
                    } else {
                        correctAngle(motors, x, y, 0.01);
                    }
                    corrections += 1;
                }
            }
        }
    }
    public static void showMaze(int[][] maze, ArrayList<MazeNode> mazeNodes){
        for (int i = 0; i<22; i++){
            for (int e = 21; e >=0; e--){
                if (maze[i][e]>0){
                    int cost = mazeNodes.get(maze[i][e]).getCost();
                    System.out.printf("%5s", " " + cost + " ");
                } else if(maze[i][e]==0){
                    System.out.printf("%5s", "|#|");
                } else if(maze[i][e]==-400) {
                    System.out.printf("%5s", "P");
                }else {
                    System.out.printf("%5s", "  ");
                }
            }
            System.out.println();
        }
    }
    public static boolean closeEnough(double a, double b, double acc){
        return java.lang.Math.abs(a-b)< acc;
    }
    public static boolean endOfMaze(){
        if (wantedX == endOfMazeX && wantedY == endOfMazeY){
            int totalCost = curTarget.getParent().getCost() + currentPathCost;
            curTarget.setCost(totalCost);
            shortestPath = totalCost;
            currentPathCost = 1;
            currentPosition = curTarget.getPath();
            mazeNodes.add(curTarget);
            maze[(int)(wantedX + 10.5)][(int)(wantedY + 10.5)] = mazeNodeCounter;
            mazeNodeCounter += 1;
            curTarget = null;
            Stack<MazeNode> helpStack = new Stack<>();
            while(!curLayerNodes.empty()){
                MazeNode mn = curLayerNodes.pop();
                if (mn.getParent().getCost()+java.lang.Math.abs(mn.getParent().getX()-endOfMazeX)+java.lang.Math.abs(mn.getParent().getY()-endOfMazeY) < shortestPath){
                    helpStack.push(mn);
                }
            }
            while(!helpStack.empty()){
                MazeNode mn = helpStack.pop();
                curLayerNodes.push(mn);
            }
            while(!nextLayerNodes.empty()){
                MazeNode mn = nextLayerNodes.pop();
                if (mn.getParent().getCost()+java.lang.Math.abs(mn.getParent().getX()-endOfMazeX)+java.lang.Math.abs(mn.getParent().getY()-endOfMazeY) < shortestPath){
                    helpStack.push(mn);
                }
            }
            while(!helpStack.empty()){
                MazeNode mn = helpStack.pop();
                nextLayerNodes.push(mn);
            }
            return true;
        } else {
            return false;
        }
    }
    public static void move(int instruction, ArrayList<ArrayList<Motor>> motors){
        if (instruction == 0){
            // go forwards
            straight(motors, blockLength, true);
            changeXY(wantedDir, 1);
        } else if (instruction == 1){
            //turn right
            turn(motors, rightAngle, false);
            wantedDir = (wantedDir + 1)%4;
        } else if (instruction == 2) {
            // go backwards
            straight(motors, blockLength, false);
            changeXY(wantedDir, -1);
        } else {
            // turn left
            turn(motors, rightAngle, true);
            wantedDir = (wantedDir + 3)%4;
        }
    }
    public static void changeXY(int dir, int forward){
        if (dir == 1){
            wantedX -= forward;
        } else if (dir == 3){
            wantedX += forward;
        } else if (dir == 2){
            wantedY -= forward;
        } else {
            wantedY += forward;
        }
    }
    public static void turn(ArrayList<ArrayList<Motor>> motors, double amount, boolean left){
        if (left){
            wantedLwheelPos -= amount;
            wantedRwheelPos += amount;
        } else {
            wantedLwheelPos += amount;
            wantedRwheelPos -= amount;
        }
        setMotPos(motors, wantedLwheelPos, wantedRwheelPos);
    }
    public static void straight(ArrayList<ArrayList<Motor>> motors, double amount, boolean forward){
        if (forward){
            wantedLwheelPos += amount;
            wantedRwheelPos += amount;
        } else {
            wantedLwheelPos -= amount;
            wantedRwheelPos -= amount;
        }
        setMotPos(motors, wantedLwheelPos, wantedRwheelPos);
    }
    public static void setMotPos(ArrayList<ArrayList<Motor>> motors, double lAmount, double rAmount){
        for (Motor m : motors.get(0)){
            m.setPosition(lAmount);
        }
        for (Motor m : motors.get(1)){
            m.setPosition(rAmount);
        }
    }
    public static void correctAngle(ArrayList<ArrayList<Motor>> motors, double x, double y, double amount){
        if (java.lang.Math.abs(x) < java.lang.Math.abs(y)){
            if (x*y > 0){
                turn(motors, amount, false);
            } else {
                turn(motors, amount, true);
            }
        } else {
            if (x*y > 0){
                turn(motors, amount, true);
            } else {
                turn(motors, amount, false);
            }
        }
    }
    public static void correctPos(ArrayList<ArrayList<Motor>> motors, double x, double y, double amount){
        if (wantedDir == 1) {
            if (x > wantedX){
                straight(motors, amount, true);
            } else {
                straight(motors, amount, false);
            }
        } else if (wantedDir == 3){
            if (x < wantedX){
                straight(motors, amount, true);
            } else {
                straight(motors, amount, false);
            }
        } else if (wantedDir == 2) {
            if (y > wantedY){
                straight(motors, amount, true);
            } else {
                straight(motors, amount, false);
            }
        } else {
            if (y < wantedY){
                straight(motors, amount, true);
            } else {
                straight(motors, amount, false);
            }
        }
    }
}

class MazeNode extends Object{
    private LinkedList<Integer> path = new LinkedList<>();
    private MazeNode parent;
    private MazeNode lChild;
    private MazeNode fChild;
    private MazeNode rChild;
    private int cost = 1;
    private int dirFromParent;
    private double X;
    private double Y;
    private boolean dead = false;
    
    public MazeNode(){
        
    }
    
    public void kill(){
        this.dead = true;
        for (MazeNode mn : this.children()){
            mn.kill();
        }
    }
    
    public boolean isDead(){
        return dead;
    }
    // Getters and Setters
    public void setPath(LinkedList<Integer> p){
        this.path = p;
    }
    
    public void setParent(MazeNode x){
        this.parent = x;
    }
    
    public void setX(double x){
        this.X = x;
    }
    
    public void setY(double x){
        this.Y = x;
    }
    
    public double getX(){
        return this.X;
    }
    
    public double getY(){
        return this.Y;
    }
    
    public void setlChild(MazeNode x){
        this.lChild = x;
        x.setParent(this);
        x.setDirFromParent(3);
        for (Integer i : this.getPath()){
            x.addToPath(i);
        }
        x.addToPath(3);
        x.addToPath(0);
    }
    
    public void setfChild(MazeNode x){
        this.fChild = x;
        x.setParent(this);
        x.setDirFromParent(0);
        for (Integer i : this.getPath()){
            x.addToPath(i);
        }
        x.addToPath(0);
    }
    
    public void setrChild(MazeNode x){
        this.rChild = x;
        x.setParent(this);
        x.setDirFromParent(1);
        for (Integer i : this.getPath()){
            x.addToPath(i);
        }
        x.addToPath(1);
        x.addToPath(0);
    }
    
    public void setCost(int i){
        this.cost = i;
    }
    
    public void setDirFromParent(int i){
        this.dirFromParent = i;
    }
    
    public LinkedList<Integer> getPath(){
        LinkedList<Integer> a = new LinkedList<>();
        for (Integer i : this.path){
            a.add(i);
        }
        return a;
    }
    
    public MazeNode getParent(){
        return this.parent;
    }
    
    public MazeNode getlChild(){
        return this.lChild;
    }
    
    public MazeNode getfChild(){
        return this.fChild;
    }
    
    public MazeNode getrChild(){
        return this.rChild;
    }
    
    public int getCost(){
        return this.cost;
    }
    
    public int getDirFromParent(){
        return this.dirFromParent;
    }
    
    public ArrayList<MazeNode> children(){
        ArrayList<MazeNode> a = new ArrayList<>();
        if (this.rChild != null){
          a.add(this.rChild);
        }
        if (this.fChild != null){
          a.add(this.fChild);
        }
        if (this.lChild != null){
          a.add(this.lChild);
        }
        return a;
    }
    
    public void addToPath(int i){
        this.path.add(i);
    }
}