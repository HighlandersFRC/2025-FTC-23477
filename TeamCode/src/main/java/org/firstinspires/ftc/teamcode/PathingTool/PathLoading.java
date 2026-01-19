package org.firstinspires.ftc.teamcode.PathingTool;

import android.content.Context;
import android.util.Log;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;

public class PathLoading {

    private static JSONObject jsonPathData;

    public PathLoading(Context context, String pathFileName) {
        try {
           jsonPathData = loadJSONFromAsset(context, pathFileName);
        } catch (IOException | JSONException e) {
            Log.e("JSON_LOAD", "Failed to load JSON file", e);
            // Handle error gracefully: show a toast, use fallback data, or stop execution
        }

    }

    private JSONObject loadJSONFromAsset(Context context, String pathFileName) throws IOException, JSONException {
        InputStream inputStream = context.getAssets().open(pathFileName);
        BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));
        StringBuilder sb = new StringBuilder();
        String line;
        while ((line = reader.readLine()) != null) {
            sb.append(line);
        }
        reader.close();
        inputStream.close();

        return new JSONObject(sb.toString());
    }


    public static JSONObject getJsonPathData() {
        return jsonPathData;
    }
}