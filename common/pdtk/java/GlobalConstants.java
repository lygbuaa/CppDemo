import android.content.res.AssetManager;
import android.system.Os;
import android.util.Log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;

public class GlobalConstants {
    private final static String TAG = "GlobalConstants";
    public static final String CAPTURE_PATH_ = "/storage/emulated/0/pdtk/capture/";
    public static final String RECORD_PATH_ = "/storage/emulated/0/pdtk/record/";
    public static final String ASSETS_PATH_ = "/storage/emulated/0/pdtk/assets/";

    public static boolean copyAssets(AssetManager assetManager, final String targetPath, final String path){
        String assets[] = null;
        try {
            assets = assetManager.list(path);
            if (assets.length == 0) {
                Log.i(TAG, String.format("[copyAssets] copying %s", path));
                copyAssetFile(assetManager, targetPath, path);
            } else {
                Log.i(TAG, String.format("[copyAssets] traverse %s", path));
                String fullPath = targetPath + path;
                File dir = new File(fullPath);
                if (!dir.exists())
                    dir.mkdirs();
                for (int i = 0; i < assets.length; ++i) {
                    copyAssets(assetManager, targetPath, path + "/" + assets[i]);
                }
            }
        } catch (Exception e) {
            Log.e(TAG, String.format("[copyAssets] %s exception: %s", path, e.getMessage()));
            return false;
        }
        return true;
    }

    public static boolean copyAssetFile(AssetManager assetManager, final String targetPath, final String filename) {
        InputStream in = null;
        OutputStream out = null;
        try {
            in = assetManager.open(filename);
            String newFileName = targetPath + filename;
            out = new FileOutputStream(newFileName);

            byte[] buffer = new byte[1024];
            int read;
            while ((read = in.read(buffer)) != -1) {
                out.write(buffer, 0, read);
            }
            in.close();
            out.flush();
            out.close();
        } catch (Exception e) {
            Log.e(TAG, String.format("[copyAssetFile] %s exception: %s", filename, e.getMessage()));
            return false;
        }
        return true;
    }

    public static void setEnvPath(){
        String snpePath = ASSETS_PATH_ + "snpe" + ";/system/lib/rfsa/adsp;/system/vendor/lib/rfsa/adsp;/dsp";
        try{
            Os.setenv("ADSP_LIBRARY_PATH", snpePath, true);
            Log.i(TAG, "set ADSP_LIBRARY_PATH: " + snpePath);
        }catch(Exception e){
            Log.e(TAG, "setEnvPath exception: " + e.toString());
        }
    }
}
