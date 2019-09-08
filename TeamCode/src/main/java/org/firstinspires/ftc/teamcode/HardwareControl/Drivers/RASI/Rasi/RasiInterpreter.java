package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.RASI.Rasi;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.RASI.RasiCommands.RasiCommands;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

/**
 * Main way to run RASI files from inside java
 *
 * construct with
 * RasiInterpreter rasi = new RasiInterpreter("Path/to/file", "filename.rasi", this);
 * (this is a LinearOpMode instance)
 * call rasi.runRasiActually() to run the file.
 * */
public class RasiInterpreter {

    private boolean  paramsAreNull;
    private int numberOfParams;
    private String LOG_TAG = "RasiInterpreter";
    private RasiLexer rasiLexer;
    private LinearOpMode linearOpMode;
    private HashMap<String, String> hashMap;
    private String methodString;
    private String lcString;
    private String type;
    private String command;
    private String mixedCaseString;
    private String[] parameters;
    private Object[] finalParameters;
    private StringBuilder stringBuilder;
    private HashMap<String, String[]> infoHashmap;
    private HashMap<String, Method> methodsHashMap;
    private HashMap<String, RasiCommands> rcHashMap;
    private boolean hasArguments;
    private Method method;

    private ArrayList<Method> methodQueue = new ArrayList<>();
    private ArrayList<Object[]> paramQueue = new ArrayList<>();
    private ArrayList<RasiCommands> rcQueue = new ArrayList<>();

    public RasiInterpreter(String filepath, String filename, LinearOpMode opmode, RasiCommands commands){
        this.linearOpMode = opmode;
        processRasiCommands(commands);
        rasiLexer = new RasiLexer(filepath, filename, linearOpMode);
    }

    public void processRasiCommands(RasiCommands r){
        if (hashMap == null) {
            hashMap = new HashMap<String, String>();
        }
        if (infoHashmap == null) {
            infoHashmap = new HashMap<String, String[]>();
        }
        if (methodsHashMap == null){
            methodsHashMap = new HashMap<String, Method>();
        }
        if (rcHashMap == null){
            rcHashMap = new HashMap<>();
        }

        for(int x = 0; x < r.getClass().getMethods().length; x++){ //runs for every method in the TeamRasiCommands Class
            if(r.getClass().getMethods()[x].toString().contains("RasiCommands.")){ //filters out the stuff that java puts there and hides.
                method = r.getClass().getMethods()[x];
                methodString = method.toString();
                stringBuilder = new StringBuilder(methodString); //StringBuilder to format the method text to be more usable.
                int index = 0;
                //remove spaces and close parenthesis:
                while(index < stringBuilder.length()){
                    if(stringBuilder.charAt(index) == ' ' || stringBuilder.charAt(index) == ')'){
                        stringBuilder.deleteCharAt(index);
                    }
                    else{
                        index++;
                    }
                }
                methodString = stringBuilder.toString();
                String[] tempArray = methodString.split("\\(");
                tempArray = tempArray[0].split("\\."); //set mixedCaseString to the name of the method. removes the remaining parenthesis and dots.
                mixedCaseString = tempArray[tempArray.length-1];

                if(methodString.charAt(methodString.length()-1)!= '(') {
                    //make a string of the arguments to the method
                    parameters = methodString.split("\\(");
                    parameters = parameters[1].split(",");
                    hasArguments = true;
                    numberOfParams = parameters.length;
                }
                else{
                    hasArguments= false;
                    parameters = null;
                    numberOfParams = 0;
                }
                lcString = mixedCaseString.toLowerCase();
                hashMap.put(lcString, mixedCaseString);
                infoHashmap.put(mixedCaseString, parameters);
                methodsHashMap.put(mixedCaseString, method);
                rcHashMap.put(mixedCaseString, r);
            }
        }
    }

    public RasiInterpreter(String filepath, String filename, LinearOpMode opmode){
        this.linearOpMode = opmode;
        rasiLexer = new RasiLexer(filepath, filename, linearOpMode);
        processRasiCommands(new RasiCommands(opmode));
    }

    public void runRasiActually() {
        command = rasiLexer.getCommand();
        while (!rasiLexer.fileEnded && !linearOpMode.isStopRequested()) {
            Log.d(LOG_TAG, "Got command " + command);
            if (infoHashmap.get(hashMap.get(command.toLowerCase())) != null) {
                paramsAreNull = false;
            } else {
                paramsAreNull = true;
            }
            if (!paramsAreNull) {
                finalParameters = new Object[infoHashmap.get(hashMap.get(command.toLowerCase())).length];
                for (int index = 0; index < finalParameters.length; index++) {
                    type = infoHashmap.get(hashMap.get(command.toLowerCase()))[index];
                    switch (type) {
                        case "int":
                            finalParameters[index] = Integer.valueOf(rasiLexer.parameters[index+1]);
                            break;
                        case "char":
                            finalParameters[index] = rasiLexer.parameters[index+1].charAt(0);
                            break;
                        case "long":
                            finalParameters[index] = Long.valueOf(rasiLexer.parameters[index+1]);
                            break;
                        case "float":
                            finalParameters[index] = Float.valueOf(rasiLexer.parameters[index+1]);
                            break;
                        case "double":
                            finalParameters[index] = Double.valueOf(rasiLexer.parameters[index+1]);
                            break;
                        case "java.lang.String":
                            finalParameters[index] = rasiLexer.parameters[index+1];
                            break;
                        case "boolean":
                            finalParameters[index] = Boolean.valueOf(rasiLexer.parameters[index+1]);
                            break;
                    }
                }
            } else {
                finalParameters = null;
            }
            if(finalParameters != null){
                try {
                    String command_lower = command.toLowerCase();
                    String hash = hashMap.get(command_lower);
                    Method method = methodsHashMap.get(hash);
                    RasiCommands rc = rcHashMap.get(hash);
                    appendMethod(method, finalParameters, rc);
                    method.invoke(rc, finalParameters);
                    Log.d(LOG_TAG, "Invoked command " + method + " with params " + Arrays.toString(finalParameters));
                } catch (Exception e){
                    Log.d(LOG_TAG, "Failed on command" + command);
                }
            } else {
                try {
                    String command_lower = command.toLowerCase();
                    String hash = hashMap.get(command_lower);
                    Method method = methodsHashMap.get(hash);
                    RasiCommands rc = rcHashMap.get(hash);
                    appendMethod(method, finalParameters, rc);
                    method.invoke(rc, finalParameters);
                    Log.d(LOG_TAG, "Invoked command " + method);
                } catch (Exception e) {
                    //Log.d(LOG_TAG, "Failed on Command " + command);
                }
            }
            command = rasiLexer.getCommand();

        }
    }

    public void runRasi(){
        runRasiActually();
        run();
    }

    public void run(){
        Method currmethod;
        Object[] param;
        RasiCommands rasiCommands;
        while (!methodQueue.isEmpty() && !paramQueue.isEmpty()){
            currmethod   = methodQueue.get(0);
            param        = paramQueue.get(0);
            rasiCommands = rcQueue.get(0);
            Log.d(LOG_TAG, "Got command " + currmethod);
            try {
                currmethod.invoke(rasiCommands, param);
                Log.d(LOG_TAG, "Finished invoking command " + currmethod + " with params " + Arrays.toString(param) + " and rasi commands " + rasiCommands);
            }catch (Exception e){
                //Log.e(LOG_TAG, "Exception occured in Run", e);
            }
            rcQueue.remove(0);
            methodQueue.remove(0);
            paramQueue.remove(0);
        }
    }

    public void setTags(String[] Tags){ //sets the rasi tags
        rasiLexer.setTags(Tags);
    }
    public void addTag(String tag){ //adds a rasi tag
        rasiLexer.addTag(tag);
    }
    public void removeTag(String tag){ // removes a rasi tag
        rasiLexer.removeTag(tag);
    }

    private void appendMethod(Method m, Object[] p, RasiCommands rc){
        methodQueue.add(m);
        paramQueue.add(p);
        rcQueue.add(rc);
    }
}
