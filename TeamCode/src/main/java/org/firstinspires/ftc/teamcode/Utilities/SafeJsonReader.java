package org.firstinspires.ftc.teamcode.Utilities;


import android.util.Log;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Iterator;
import java.io.FileWriter;

/*
  where are the file on the phone?
        storage / emulated / FIRST / team9773 / json18
        1) open terminal tab on android studio
        2) get to the right dir on the computer, for example
        cd TeamCode/src/main/java/org/firstinspires/ftc/teamcode/json/
        3) push a file to the phone:
        adb push myfile.json /sdcard/FIRST/team9773/json19/
        location of adb on mac: $HOME/Library/Android/sdk/platform-tools
          where you can get the $HOME value by typing "echo $HOME" in a terminal
          export PATH=$PATH:$HOME/Library/Android/sdk/platform-tools
        4) get a file from the phone
        adb pull  /sdcard/FIRST/team9773/json19/myfile.json
*/
public class SafeJsonReader {
    private static final String baseDir = "/sdcard/FIRST/team9773/json20"; // must end with a name
    private static final String TAG = "ftc9773 SafeJasonReader";
    private static final boolean DEBUG = false;

    private String fileName;
    private boolean modified;
    private String jsonStr;
    public JSONObject jsonRoot;

    private String FullName() {
        return baseDir + "/" + this.fileName + ".json";
    }

    /**
     * Constructor that creates the SafeJsonReader object.
     * the filename should just be the local name. Automatically appends the root path to the
     * given filename, as well as the .json extension.
     * @param fileName the local name of the file. should not have whitespace
     */
    public SafeJsonReader(String fileName) {
        this.fileName = fileName;
        this.modified = false;
        // read file
        String filePath = FullName();
        if (DEBUG) Log.d(TAG, "try to read json file " + filePath);
        FileReader fileReader = null;
        BufferedReader bufReader = null;
        StringBuilder strBuilder = new StringBuilder();
        String line = null;
        // If the given file path does not exist, give an error showing the JSON file not able to open
        try {
            fileReader = new FileReader(filePath);
            bufReader = new BufferedReader(fileReader);
        }
        catch (IOException e) {
            Log.e(TAG, "Error while trying to open the json file" + filePath, e);
        }

        // Read the file and append to the string builder
        try {
            while ((line = bufReader.readLine()) != null) {
                strBuilder.append(line);
            }
            // Now initialize the main variable that holds the entire json config
            this.jsonStr = new String(strBuilder);
        }
        catch (IOException e) {
            Log.e(TAG, "Error while trying to reading the json file" + filePath, e);
        }

        // construct the json root object
        try {
            this.jsonRoot = new JSONObject(jsonStr);
        }
        catch (JSONException e) {
            Log.e(TAG, "Error while trying to parsing the json file" + fileName, e);
        }

        // cleanup file
        try {
            fileReader.close();
        } catch (IOException e) {
            Log.e(TAG, "Error while trying to closing the file" + filePath, e);
        }
    }

    /**
     * Writes the updated version of the file to storage.
     * <p> it is necessary to call this method in order to store a modified
     * version of a value.
     * </p>
     * @return success value - returns true if a success, false if operation failed.
     */
    public boolean updateFile()
    {
        if (! this.modified) return true;

        // file path (same as reading)
        String filePath = FullName();
        if (DEBUG) Log.d(TAG, "try to write json file " + filePath);
        // open file
        FileWriter fileWriter = null;
        try {
            fileWriter = new FileWriter(filePath);
        }
        catch (IOException e) {
            Log.e(TAG, "Error while trying to open the json file" + this.fileName + " in write mode", e);
            return false;
        }
        // write file
        try {
            fileWriter.write(this.jsonRoot.toString());
        }
        catch (IOException e) {
            Log.e(TAG, "Error while trying to write the json file" + this.fileName, e);
            return false;
        }
        // cleanup file
        try {
            fileWriter.close();
        } catch (IOException e) {
            Log.e(TAG, "Error while trying to closing the file" + filePath, e);
            return false;
        }
        this.modified = false;
        return true;

    }

    // This is a private class method which read key while ignoring the case
    private static String getRealKeyIgnoreCase(JSONObject jobj, String key) throws JSONException {
        Iterator<String> iter = jobj.keys();
        while (iter.hasNext()) {
            String key1 = iter.next();
            if (key1.equalsIgnoreCase(key)) {
                return (key1);
            }
        }
        return null;
    }

    /**
     *  reads a String from the Json file; ignores capitals in the name of the file, and automatically
     *  matches them to the correct key regardless of case
     *
     *  @param obj The JSON object that is being read from.
     *  @param name the non-case sensitive key that the value is stored at.
     *  @return The string at the key, or null if no value is found there.
     */
    public String getString(JSONObject obj, String name)
    {
        String value=null;
        try {
            String key = getRealKeyIgnoreCase(obj, name);
            value = obj.getString(key);
        } catch (JSONException e) {
            Log.e(TAG, "Error while getting string value for key " + name + " in json file " + this.fileName, e);
        }
        if (DEBUG) {
            if (value!=null) Log.d(TAG, "read string for key " + name + " and got " + value);
            else Log.e(TAG, "read string for key " + name + " and got null");
        }
        return (value);
    }
    /**
     *  reads a String from the Json file; ignores capitals in the name of the file, and automatically
     *  matches them to the correct key regardless of case
     *
     *  @param obj The JSON object that is being read from.
     *  @param name the non-case sensitive key that the value is stored at.
     *  @param defaultVal the defalut value. This will be returned if an error occurs in reading the value
     *  @return The string at the key, or the default value if no value is found there.
     */
    public String getString(JSONObject obj, String name, String defaultVal)
    {
        String value=defaultVal;
        try {
            String key = getRealKeyIgnoreCase(obj, name);
            value = obj.getString(key);
        } catch (JSONException e) {
            Log.e(TAG, "Error while getting string value for key " + name + " in json file " + this.fileName, e);
        }
        if (DEBUG) {
            if (value!=null) Log.d(TAG, "read string for key " + name + " and got " + value);
            else Log.e(TAG, "read string for key " + name + " and got null");
        }
        return (value);
    }


    /**
     * Tells the program to modify the value of a string. Needs to be used in conjunction with the
     * readSensors file method to readSensors files.
     * @param name the non-case sensitive key that the value being modified is stored in the JSON file.
     * @param newValue The updated string to be used.
     * @return success value - returns true if a success, false if operation failed.
     */
    public boolean modifyString(String name, String newValue)
    {
        try {
            String key = getRealKeyIgnoreCase(jsonRoot, name);
            String oldValue = this.jsonRoot.getString(key);
            if (! oldValue.equals(newValue)) {
                this.jsonRoot.put(key, newValue);
                this.modified = true;
                if (DEBUG) Log.d(TAG, "write string for key " + name + " with new value " + newValue);
            }
        } catch (JSONException e) {
            Log.d(TAG, "Error while setting string value for key " + name + " to " + newValue + " in json file " + this.fileName, e);
            return false;
        }
        return true;
    }
    /**
     *  reads an integer from the Json file; ignores capitals in the name of the file, and automatically
     *  matches them to the correct key regardless of case
     *
     *  @param obj The JSON object that is being read from.
     *  @param name the non-case sensitive key that the value is stored at.
     *  @return The value read at the key, or 0 if no value is found there.
     */
    public int getInt(JSONObject obj, String name) {
        int value=0;
        try {
            String key = getRealKeyIgnoreCase(obj, name);
            value = obj.getInt(key);
        } catch (JSONException e) {
            Log.e(TAG, "Error while getting int value for key " + name + " in json file " + this.fileName, e);
        }
        if (DEBUG) Log.d(TAG, "read int for key " + name + " and got " + value);
        return (value);
    }
    /**
     *  reads an integer from the Json file; ignores capitals in the name of the file, and automatically
     *  matches them to the correct key regardless of case
     *
     *  @param obj The JSON object that is being read from.
     *  @param name the non-case sensitive key that the value is stored at.
     *  @param defaultVal the defalut value. This will be returned if an error occurs in reading the value
     *  @return The value read at the key, or the default value if no value is found there.
     */
    public int getInt(JSONObject obj, String name, int defaultVal) {
        int value=defaultVal;
        try {
            String key = getRealKeyIgnoreCase(obj, name);
            value = obj.getInt(key);
        } catch (JSONException e) {
            Log.e(TAG, "Error while getting int value for key " + name + " in json file " + this.fileName, e);
        }
        if (DEBUG) Log.d(TAG, "read int for key " + name + " and got " + value);
        return (value);
    }

    /**
     * Tells the program to modify the value of an int. Needs to be used in conjunction with the
     * readSensors file method to readSensors files.
     * @param name the non-case sensitive key that the value being modified is stored in the JSON file.
     * @param newValue The updated integer value to be stored.
     * @return success value - returns true if a success, false if operation failed.
     */
    public boolean modifyInt(String name, int newValue) {
        try {
            String key = getRealKeyIgnoreCase(this.jsonRoot, name);
            int oldValue = this.jsonRoot.getInt(key);
            if (oldValue != newValue) {
                this.jsonRoot.put(key, newValue);
                this.modified = true;
                if (DEBUG) Log.d(TAG, "write int for key " + name + " with new value " + newValue);

            }
        } catch (JSONException e) {
            Log.e(TAG, "Error while modifying int value for key " + name + " to " + newValue + " in json file " + this.fileName, e);
            return false;
        }
        return true;
    }


    /**
     *  reads a Double from the Json file; ignores capitals in the name of the file, and automatically
     *  matches them to the correct key regardless of case
     *
     *  @param obj The JSON object that is being read from.
     *  @param name the non-case sensitive key that the value is stored at.
     *  @return The value read at the key, or 0.0 if no value is found there.
     */
    public double getDouble(JSONObject obj, String name) {
        String key;
        double value=0.0;
        try {
            key = getRealKeyIgnoreCase(obj, name);
            value = obj.getDouble(key);
        } catch (JSONException e) {
            Log.e(TAG, "Error while getting double value for key " + name + " in json file " + this.fileName, e);
        }
        if (DEBUG) Log.d(TAG, "read double for key " + name + " and got " + value);
        return (value);
    }

    /**
     *  reads a Double from the Json file; ignores capitals in the name of the file, and automatically
     *  matches them to the correct key regardless of case
     *
     *  @param obj The JSON object that is being read from.
     *  @param name the non-case sensitive key that the value is stored at.
     *  @param defaultVal the default value. This will be returned if an error occurs in the read process
     *  @return The value read at the key, or the default value if no value is found there.
     */
    public double getDouble(JSONObject obj, String name,double defaultVal) {
        String key;
        double value= defaultVal;
        try {
            key = getRealKeyIgnoreCase(obj, name);
            value = obj.getDouble(key);
        } catch (JSONException e) {
            Log.e(TAG, "Error while getting double value for key " + name + " in json file " + this.fileName, e);
        }
        if (DEBUG) Log.d(TAG, "read double for key " + name + " and got " + value);
        return (value);
    }

    /**
     * Tells the program to modify the value of a double. Needs to be used in conjunction with the
     * readSensors file method to readSensors files.
     * @param name the non-case sensitive key that the value being modified is stored in the JSON file.
     * @param newValue The updated double being stored.
     * @return success value- returns true if a success, false if otherwise
     *
     */
    public boolean modifyDouble(String name, double newValue) {
        try {
            String key = getRealKeyIgnoreCase(this.jsonRoot, name);
            double oldValue = this.jsonRoot.getDouble(key);
            if (oldValue != newValue) {
                this.jsonRoot.put(key, newValue);
                this.modified = true;
                if (DEBUG) Log.d(TAG, "write double for key " + name + " with new value " + newValue);
            }
        } catch (JSONException e) {
            Log.e(TAG, "Error while modifying double value for key " + name + " to " + newValue + " in json file " + this.fileName, e);
            return false;
        }
        return true;
    }

    /**
     *  reads a boolean from the Json file; ignores capitals in the name of the file, and automatically
     *  matches them to the correct key regardless of case
     *
     *  @param obj The JSON object that is being read from.
     *  @param name the non-case sensitive key that the value is stored at.
     *  @return The value read at the key, or False if no value is found there.
     */
    public boolean getBoolean(JSONObject obj, String name) {
        String key;
        boolean value=false;
        try {
            key = getRealKeyIgnoreCase(obj, name);
            value = obj.getBoolean(key);
        } catch (JSONException e) {
            Log.e(TAG, "Error while getting boolean value for key " + name + " in json file " + this.fileName, e);
        }
        if (DEBUG) Log.d(TAG, "read boolean for key " + name + " and got " + value);
        return (value);
    }

    /**
     *  reads a boolean from the Json file; ignores capitals in the name of the file, and automatically
     *  matches them to the correct key regardless of case
     *
     *  @param obj The JSON object that is being read from.
     *  @param name the non-case sensitive key that the value is stored at.
     *  @param defaultValue the defalut value. This will be returned if an error occurs while reading the value
     *  @return The value read at the key, or the default value if no value is found there.
     */
    public boolean getBoolean(JSONObject obj, String name, boolean defaultValue) {
        String key;
        boolean value=defaultValue;
        try {
            key = getRealKeyIgnoreCase(obj, name);
            value = obj.getBoolean(key);
        } catch (JSONException e) {
            Log.e(TAG, "Error while getting boolean value for key " + name + " in json file " + this.fileName, e);
        }
        if (DEBUG) Log.d(TAG, "read boolean for key " + name + " and got " + value);
        return (value);
    }


    /**
     * Tells the program to modify the value of a boolean. Needs to be used in conjunction with the
     * readSensors file method to readSensors files.
     * @param name the non-case sensitive key that the value being modified is stored in the JSON file
     * @param newValue The updated boolean value to be stored.
     * @return returns true if a success; false otherwise
     */
    public boolean modifyBoolean(String name, boolean newValue) {
        try {
            String key = getRealKeyIgnoreCase(this.jsonRoot, name);
            boolean oldValue = this.jsonRoot.getBoolean(key);
            if (oldValue != newValue) {
                this.jsonRoot.put(key, newValue);
                this.modified = true;
                if (DEBUG) Log.d(TAG, "write boolean for key " + name + " with new value " + newValue);

            }
        } catch (JSONException e) {
            Log.e(TAG, "Error while modifying boolean value for key " + name + " to " + newValue + " in json file " + this.fileName, e);
            return false;
        }
        return true;
    }

    /**
     *  reads a JSON object from the Json file; ignores capitals in the name of the file, and automatically
     *  matches them to the correct key regardless of case
     *
     *  @param obj The JSON object that is being read from.
     *  @param name the non-case sensitive key that the value is stored at.
     *  @return The value read at the key, or null if no value is found there.
     */
    public JSONObject getJSONObject(JSONObject obj, String name) {
        String key;
        JSONObject value=null;
        try {
            key = getRealKeyIgnoreCase(obj, name);
            value = obj.getJSONObject(key);
        } catch (JSONException e) {
            Log.e(TAG, "Error while getting json object value for key " + name + " in json file " + this.fileName, e);
        }
        return (value);
    }

    /**
     *  reads a JSON array from the Json file; ignores capitals in the name of the file, and automatically
     *  matches them to the correct key regardless of case
     *
     *  @param obj The JSON object that is being read from.
     *  @param name the non-case sensitive key that the value is stored at.
     *  @return The value read at the key, or null if no value is found there.
     */
    public JSONArray getJSONArray(JSONObject obj, String name) {
        String key;
        JSONArray value=null;
        try {
            key = getRealKeyIgnoreCase(obj, name);
            value = obj.getJSONArray(key);
        } catch (JSONException e) {
            Log.e(TAG, "Error while getting json array value for key " + name + " in json file " + this.fileName, e);
        }
        return (value);
    }

    // aliases

    /**
     *  reads a String from the Json file specified in the constructor; ignores capitals in the name
     *  of the file, and automatically matches them to the correct key regardless of case
     *
     *  @param name the non-case sensitive key that the value is stored at.
     *  @return The string at the key, or null if no value is found there.
     */
    public String getString(String name) { return getString(jsonRoot, name); }
    /**
     *  reads a String from the Json file specified in the constructor; ignores capitals in the name
     *  of the file, and automatically matches them to the correct key regardless of case
     *
     *  @param name the non-case sensitive key that the value is stored at.
     *  @param defaultValue the defalut value. This will be returned if an error occurs while reading the value
     *  @return The string at the key, or the default value if no value is found there.
     */
    public String getString(String name, String defaultValue) { return getString(jsonRoot, name,defaultValue); }


    /**
     *  reads a Int from the Json file specified in the constructor; ignores capitals in the name
     *  of the file, and automatically matches them to the correct key regardless of case
     *
     *  @param name the non-case sensitive key that the value is stored at.
     *  @return The integer value at the key, or 0.0 if no value is found there.
     */
    public int getInt(String name) { return getInt(jsonRoot, name); }
    /**
     *  reads an int from the Json file specified in the constructor; ignores capitals in the name
     *  of the file, and automatically matches them to the correct key regardless of case
     *
     *  @param name the non-case sensitive key that the value is stored at.
     *  @param defaultValue the defalut value. This will be returned if an error occurs while reading the value
     *  @return The int at the key, or the default value if no value is found there.
     */
    public int getInt(String name, int defaultValue) { return getInt(jsonRoot, name,defaultValue); }

    /**
     *  reads a Double from the Json file specified in the constructor; ignores capitals in the name
     *  of the file, and automatically matches them to the correct key regardless of case
     *
     *  @param name the non-case sensitive key that the value is stored at.
     *  @return The double value at the key, or 0.0 if no value is found there.
     */
    public double getDouble(String name) { return getDouble(jsonRoot, name); }
    /**
     *  reads a Double from the Json file specified in the constructor; ignores capitals in the name
     *  of the file, and automatically matches them to the correct key regardless of case
     *
     *  @param name the non-case sensitive key that the value is stored at.
     *  @param defaultValue the defalut value. This will be returned if an error occurs while reading the value
     *  @return The double value at the key, or the default value if an error occurs
     */
    public double getDouble(String name,double defaultValue) { return getDouble(jsonRoot, name, defaultValue); }


    /**
     *  reads a Boolean value from the Json file specified in the constructor; ignores capitals in the name
     *  of the file, and automatically matches them to the correct key regardless of case
     *
     *  @param name the non-case sensitive key that the value is stored at.
     *  @return The boolean value at the key, or false if no value is found there.
     */
    public boolean getBoolean(String name) { return getBoolean(jsonRoot, name); }

    /**
     *  reads a Boolean value from the Json file specified in the constructor; ignores capitals in the name
     *  of the file, and automatically matches them to the correct key regardless of case
     *
     *  @param name the non-case sensitive key that the value is stored at.
     *  @param defaultValue the defalut value. This will be returned if an error occurs while reading the value
     *  @return The boolean value at the key, or the default value if no value is found there.
     */
    public boolean getBoolean(String name, boolean defaultValue) { return getBoolean(jsonRoot, name,defaultValue); }

    /**
     *  reads a JSON object from the Json file specified in the constructor; ignores capitals in the name
     *  of the file, and automatically matches them to the correct key regardless of case
     *
     *  @param name the non-case sensitive key that the value is stored at.
     *  @return The JSON object at the key, or null if no value is found there.
     */
    public JSONObject getJSONObject(String name) { return getJSONObject(jsonRoot, name); }
    /**
     *  reads a JSON array from the Json file specified in the constructor; ignores capitals in the name
     *  of the file, and automatically matches them to the correct key regardless of case
     *
     *  @param name the non-case sensitive key that the value is stored at.
     *  @return The JSON array at the key, or null if no value is found there.
     */
    public JSONArray getJSONArray(String name) { return getJSONArray(jsonRoot, name); }

}
