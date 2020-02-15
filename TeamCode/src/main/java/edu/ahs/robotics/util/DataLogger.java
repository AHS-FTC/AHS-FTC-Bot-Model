package edu.ahs.robotics.util;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class DataLogger extends Logger {
    private ArrayList<String> categories;
    private Map<String, ArrayList<String>> entriesByCategory;
    private int lastLine = 0;

    public DataLogger(String fileName, String key) {
        super(fileName, key, ".csv");
        categories = new ArrayList<>();
        entriesByCategory = new HashMap<>();
    }

    private void writeCats() throws IOException {
        fileWriter.append("Time, ");
        int i = 0;
        for(String cat : categories){ // write the categories first
            fileWriter.append(cat);
            if(i < categories.size() - 1){
                fileWriter.append(", ");
            }
            i++;
        }
        fileWriter.append("\n");
    }

    public void append(String category, String data) {
        ArrayList<String> dataList = entriesByCategory.get(category);
        if (dataList == null) {
            dataList = new ArrayList<>();
            categories.add(category);
            entriesByCategory.put(category,dataList);
        }
        dataList.add(data);
    }

    public void writeLine(){
        try {
            if(firstLine){
                writeCats();
                firstLine = false;
            }
            fileWriter.append(String.valueOf(System.currentTimeMillis() - startTime) + ", ");
            for (int i = 0; i < categories.size(); i++) {
                String category = categories.get(i);
                List<String> list = entriesByCategory.get(category);
                if (lastLine < list.size()) {
                    fileWriter.append(list.get(lastLine));
                } else {
                    fileWriter.append(" ");
                }
                if (i < categories.size() - 1) {
                    fileWriter.append(", ");
                }
            }
            fileWriter.append("\n");
            lastLine++;
            fileWriter.flush();
        } catch (Exception e){
            throw new Warning(e.getMessage());
        }
    }
}
