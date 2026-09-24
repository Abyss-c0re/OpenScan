package dev.lab.openscan;

import java.io.BufferedInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

/** STL, OBJ, or ASCII PLY, including a point cloud with no faces. */
public final class MeshFile {
    public float[] xyz = new float[0];
    public float[] nrm = new float[0];
    public int[] idx = new int[0];
    public boolean pointsOnly;

    public static MeshFile load(File file) throws Exception {
        try (InputStream in = new FileInputStream(file)) {
            return load(in, file.getName());
        }
    }

    public static MeshFile load(InputStream raw, String name) throws Exception {
        ByteArrayOutputStream bos = new ByteArrayOutputStream();
        byte[] buf = new byte[16384];
        int n;
        BufferedInputStream in = new BufferedInputStream(raw);
        while ((n = in.read(buf)) > 0) bos.write(buf, 0, n);
        byte[] data = bos.toByteArray();
        int headN = Math.min(data.length, 256);
        String head = new String(data, 0, headN, StandardCharsets.US_ASCII);
        if (head.startsWith("ply")) return parsePly(new String(data, StandardCharsets.US_ASCII));
        if (head.startsWith("solid")) return parseAsciiStl(new String(data, StandardCharsets.US_ASCII));
        if (head.startsWith("v ") || head.startsWith("vn ") || head.startsWith("vt ")
                || (head.startsWith("#") && (head.contains("\nv ") || head.contains("\r\nv "))))
            return parseObj(new String(data, StandardCharsets.US_ASCII));
        if (data.length >= 84) {
            int tris = ByteBuffer.wrap(data, 80, 4).order(ByteOrder.LITTLE_ENDIAN).getInt();
            long need = 84L + (long) tris * 50L;
            if (tris >= 0 && data.length >= need) return parseBinaryStl(data, tris);
        }
        String lower = name == null ? "" : name.toLowerCase();
        String text = new String(data, StandardCharsets.US_ASCII);
        if (lower.endsWith(".obj") || text.contains("\nv ") || text.startsWith("v ")) return parseObj(text);
        if (lower.endsWith(".ply") || text.startsWith("ply")) return parsePly(text);
        return parseAsciiStl(text);
    }

    private static MeshFile parseBinaryStl(byte[] data, int tris) {
        MeshFile m = new MeshFile();
        if (tris <= 0) return m;
        m.xyz = new float[tris * 9];
        m.nrm = new float[tris * 9];
        m.idx = new int[tris * 3];
        ByteBuffer bb = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);
        int off = 84;
        for (int t = 0; t < tris; t++) {
            float nx = bb.getFloat(off);
            float ny = bb.getFloat(off + 4);
            float nz = bb.getFloat(off + 8);
            for (int k = 0; k < 3; k++) {
                int base = t * 9 + k * 3;
                m.xyz[base] = bb.getFloat(off + 12 + k * 12);
                m.xyz[base + 1] = bb.getFloat(off + 16 + k * 12);
                m.xyz[base + 2] = bb.getFloat(off + 20 + k * 12);
                m.nrm[base] = nx;
                m.nrm[base + 1] = ny;
                m.nrm[base + 2] = nz;
                m.idx[t * 3 + k] = t * 3 + k;
            }
            off += 50;
        }
        return m;
    }

    private static MeshFile parseAsciiStl(String text) {
        List<Float> v = new ArrayList<>();
        List<Float> nn = new ArrayList<>();
        float nx = 0, ny = 0, nz = 1;
        for (String line : text.split("\n")) {
            String s = line.trim();
            if (s.startsWith("facet normal")) {
                String[] p = s.split("\\s+");
                if (p.length >= 5) {
                    nx = f(p[2]); ny = f(p[3]); nz = f(p[4]);
                }
            } else if (s.startsWith("vertex")) {
                String[] p = s.split("\\s+");
                if (p.length >= 4) {
                    v.add(f(p[1])); v.add(f(p[2])); v.add(f(p[3]));
                    nn.add(nx); nn.add(ny); nn.add(nz);
                }
            }
        }
        return fromTriangles(v, nn);
    }

    private static MeshFile parseObj(String text) {
        List<Float> verts = new ArrayList<>();
        List<Integer> faces = new ArrayList<>();
        for (String line : text.split("\n")) {
            String s = line.trim();
            if (s.startsWith("v ")) {
                String[] p = s.split("\\s+");
                if (p.length >= 4) {
                    verts.add(f(p[1])); verts.add(f(p[2])); verts.add(f(p[3]));
                }
            } else if (s.startsWith("f ")) {
                String[] p = s.split("\\s+");
                int[] ids = new int[p.length - 1];
                for (int i = 1; i < p.length; i++) {
                    String tok = p[i].split("/")[0];
                    int id = Integer.parseInt(tok);
                    if (id < 0) id = verts.size() / 3 + id;
                    else id = id - 1;
                    ids[i - 1] = id;
                }
                for (int i = 1; i + 1 < ids.length; i++) {
                    faces.add(ids[0]); faces.add(ids[i]); faces.add(ids[i + 1]);
                }
            }
        }
        MeshFile m = new MeshFile();
        m.xyz = new float[verts.size()];
        for (int i = 0; i < verts.size(); i++) m.xyz[i] = verts.get(i);
        m.idx = new int[faces.size()];
        for (int i = 0; i < faces.size(); i++) m.idx[i] = faces.get(i);
        m.pointsOnly = m.idx.length == 0;
        m.nrm = faceNormals(m.xyz, m.idx);
        return m;
    }

    private static MeshFile parsePly(String text) {
        String[] lines = text.split("\n");
        int nv = 0, nf = 0, header = 0;
        boolean ascii = true;
        for (int i = 0; i < lines.length; i++) {
            String s = lines[i].trim();
            if (s.startsWith("format") && s.contains("binary")) ascii = false;
            if (s.startsWith("element vertex")) nv = Integer.parseInt(s.split("\\s+")[2]);
            if (s.startsWith("element face")) nf = Integer.parseInt(s.split("\\s+")[2]);
            if (s.equals("end_header")) { header = i + 1; break; }
        }
        MeshFile m = new MeshFile();
        if (!ascii) throw new IllegalArgumentException("binary ply");
        m.xyz = new float[nv * 3];
        for (int i = 0; i < nv && header + i < lines.length; i++) {
            String[] p = lines[header + i].trim().split("\\s+");
            if (p.length < 3) continue;
            m.xyz[i * 3] = f(p[0]);
            m.xyz[i * 3 + 1] = f(p[1]);
            m.xyz[i * 3 + 2] = f(p[2]);
        }
        m.idx = new int[nf * 3];
        for (int i = 0; i < nf && header + nv + i < lines.length; i++) {
            String[] p = lines[header + nv + i].trim().split("\\s+");
            if (p.length < 4) continue;
            m.idx[i * 3] = Integer.parseInt(p[1]);
            m.idx[i * 3 + 1] = Integer.parseInt(p[2]);
            m.idx[i * 3 + 2] = Integer.parseInt(p[3]);
        }
        m.pointsOnly = nf == 0;
        m.nrm = faceNormals(m.xyz, m.idx);
        return m;
    }

    private static MeshFile fromTriangles(List<Float> v, List<Float> nn) {
        MeshFile m = new MeshFile();
        m.xyz = new float[v.size()];
        m.nrm = new float[nn.size()];
        m.idx = new int[v.size() / 3];
        for (int i = 0; i < v.size(); i++) m.xyz[i] = v.get(i);
        for (int i = 0; i < nn.size(); i++) m.nrm[i] = nn.get(i);
        for (int i = 0; i < m.idx.length; i++) m.idx[i] = i;
        return m;
    }

    private static float[] faceNormals(float[] xyz, int[] idx) {
        float[] n = new float[xyz.length];
        for (int i = 0; i + 2 < idx.length; i += 3) {
            int a = idx[i] * 3, b = idx[i + 1] * 3, c = idx[i + 2] * 3;
            if (a < 0 || c + 2 >= xyz.length) continue;
            float ax = xyz[b] - xyz[a], ay = xyz[b + 1] - xyz[a + 1], az = xyz[b + 2] - xyz[a + 2];
            float bx = xyz[c] - xyz[a], by = xyz[c + 1] - xyz[a + 1], bz = xyz[c + 2] - xyz[a + 2];
            float nx = ay * bz - az * by, ny = az * bx - ax * bz, nz = ax * by - ay * bx;
            n[a] += nx; n[a + 1] += ny; n[a + 2] += nz;
            n[b] += nx; n[b + 1] += ny; n[b + 2] += nz;
            n[c] += nx; n[c + 1] += ny; n[c + 2] += nz;
        }
        return n;
    }

    private static float f(String s) {
        try { return Float.parseFloat(s); } catch (Exception e) { return 0f; }
    }
}
