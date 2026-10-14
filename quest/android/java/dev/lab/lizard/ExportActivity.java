package dev.lab.lizard;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.widget.Toast;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.io.OutputStream;

/** Opens DocumentsUI so an export can be STL, OBJ, or PLY. */
public class ExportActivity extends Activity {
    private static final int REQ = 7;
    private String pendingFile;
    private String[] choices;

    @Override
    protected void onCreate(Bundle saved) {
        super.onCreate(saved);
        if (saved != null) pendingFile = saved.getString("file");
        if (pendingFile != null) return;
        showFormats();
    }

    @Override
    protected void onSaveInstanceState(Bundle out) {
        super.onSaveInstanceState(out);
        if (pendingFile != null) out.putString("file", pendingFile);
    }

    private void showFormats() {
        final String[] labels = new String[3];
        final String[] files = new String[3];
        int n = 0;
        n = add(labels, files, n, "STL", "lizard-model.stl");
        n = add(labels, files, n, "OBJ", "lizard-model.obj");
        if (ready("lizard-solid.ply")) n = add(labels, files, n, "PLY", "lizard-solid.ply");
        else n = add(labels, files, n, "PLY", "lizard-cloud.ply");
        if (n < 1) {
            Toast.makeText(this, "Nothing to export yet. Scan until the model is inside the cube.", Toast.LENGTH_LONG).show();
            finish();
            return;
        }
        final String[] show = new String[n];
        choices = new String[n];
        for (int i = 0; i < n; i++) {
            show[i] = labels[i];
            choices[i] = files[i];
        }
        new AlertDialog.Builder(this)
                .setTitle("Export")
                .setItems(show, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        if (choices != null && which >= 0 && which < choices.length)
                            startCreate(choices[which]);
                    }
                })
                .setOnCancelListener(new DialogInterface.OnCancelListener() {
                    @Override
                    public void onCancel(DialogInterface dialog) {
                        finish();
                    }
                })
                .show();
    }

    private int add(String[] labels, String[] files, int n, String label, String name) {
        if (!ready(name) || n >= labels.length) return n;
        labels[n] = label;
        files[n] = name;
        return n + 1;
    }

    private boolean ready(String name) {
        File f = new File(getFilesDir(), name);
        return f.isFile() && f.length() > 32;
    }

    private void startCreate(String name) {
        pendingFile = name;
        String ext = "stl";
        if (name.endsWith(".obj")) ext = "obj";
        else if (name.endsWith(".ply")) ext = "ply";
        Intent intent = new Intent(Intent.ACTION_CREATE_DOCUMENT);
        intent.addCategory(Intent.CATEGORY_OPENABLE);
        intent.setType("application/octet-stream");
        intent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_WRITE_URI_PERMISSION);
        intent.putExtra(Intent.EXTRA_TITLE, "lizard-model." + ext);
        startActivityForResult(intent, REQ);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode != REQ) return;
        if (resultCode != RESULT_OK || data == null || data.getData() == null || pendingFile == null) {
            finish();
            return;
        }
        copyTo(data.getData(), pendingFile);
    }

    private void copyTo(Uri uri, String name) {
        File src = new File(getFilesDir(), name);
        try (InputStream in = new FileInputStream(src); OutputStream out = getContentResolver().openOutputStream(uri)) {
            if (out == null) throw new IllegalStateException("uri");
            byte[] buf = new byte[16384];
            int n;
            while ((n = in.read(buf)) > 0) out.write(buf, 0, n);
            String what = name.endsWith(".obj") ? "OBJ" : name.endsWith(".ply") ? "PLY" : "STL";
            Toast.makeText(this, "Saved " + what, Toast.LENGTH_LONG).show();
        } catch (Exception e) {
            Toast.makeText(this, "DocumentsUI did not accept the file", Toast.LENGTH_LONG).show();
        }
        finish();
    }
}
