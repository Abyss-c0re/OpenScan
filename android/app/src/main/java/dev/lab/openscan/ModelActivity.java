package dev.lab.openscan;

import android.app.Activity;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.widget.Toast;

import java.io.File;
import java.io.InputStream;

/** Full-screen model. Drag to rotate, pinch to zoom. */
public class ModelActivity extends Activity {
    public static final String EXTRA_PATH = "path";
    private ModelView view;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        view = new ModelView(this);
        setContentView(view);
        String path = getIntent().getStringExtra(EXTRA_PATH);
        if (path != null) show(new File(path));
        else openPicker();
    }

    private void show(File file) {
        try {
            accept(MeshFile.load(file));
        } catch (Exception e) {
            Toast.makeText(this, "Could not read the model", Toast.LENGTH_LONG).show();
            finish();
        }
    }

    private void accept(MeshFile mesh) {
        if (mesh == null || mesh.xyz.length < 3) {
            Toast.makeText(this, "Could not read the model", Toast.LENGTH_LONG).show();
            finish();
            return;
        }
        view.setMesh(mesh);
    }

    private void openPicker() {
        Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT);
        intent.addCategory(Intent.CATEGORY_OPENABLE);
        intent.setType("*/*");
        startActivityForResult(intent, 1);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (resultCode != RESULT_OK || data == null || data.getData() == null) {
            finish();
            return;
        }
        Uri uri = data.getData();
        try (InputStream in = getContentResolver().openInputStream(uri)) {
            String name = uri.getLastPathSegment();
            if (name == null) name = "model.stl";
            accept(MeshFile.load(in, name));
        } catch (Exception e) {
            Toast.makeText(this, "Could not read the model", Toast.LENGTH_LONG).show();
            finish();
        }
    }
}
