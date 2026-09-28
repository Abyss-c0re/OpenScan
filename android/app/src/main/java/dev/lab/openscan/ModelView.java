package dev.lab.openscan;

import android.content.Context;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.util.Log;
import android.view.MotionEvent;

import java.util.Arrays;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/** Drag to turn the model. A second finger up or down zooms. */
public class ModelView extends GLSurfaceView {
    private final RendererImpl renderer = new RendererImpl();
    private float lastX, lastY;
    private float span;

    public ModelView(Context context) {
        super(context);
        setEGLContextClientVersion(2);
        setRenderer(renderer);
        setRenderMode(RENDERMODE_WHEN_DIRTY);
    }

    public void setMesh(MeshFile mesh) {
        renderer.mesh = mesh;
        renderer.uploaded = false;
        requestRender();
    }

    @Override
    public boolean onTouchEvent(MotionEvent e) {
        int action = e.getActionMasked();
        if (e.getPointerCount() >= 2) {
            float dx = e.getX(0) - e.getX(1);
            float dy = e.getY(0) - e.getY(1);
            float d = (float) Math.hypot(dx, dy);
            if (action == MotionEvent.ACTION_MOVE && span > 1f) {
                renderer.scale *= d / span;
                if (renderer.scale < 0.2f) renderer.scale = 0.2f;
                if (renderer.scale > 8f) renderer.scale = 8f;
                requestRender();
            }
            span = d;
            lastX = e.getX(0);
            lastY = e.getY(0);
            return true;
        }
        span = 0;
        if (action == MotionEvent.ACTION_DOWN || action == MotionEvent.ACTION_POINTER_UP) {
            lastX = e.getX();
            lastY = e.getY();
            return true;
        }
        if (action == MotionEvent.ACTION_MOVE) {
            renderer.yaw += (e.getX() - lastX) * 0.01f;
            renderer.pitch += (e.getY() - lastY) * 0.01f;
            if (renderer.pitch < -1.4f) renderer.pitch = -1.4f;
            if (renderer.pitch > 1.4f) renderer.pitch = 1.4f;
            lastX = e.getX();
            lastY = e.getY();
            requestRender();
        }
        return true;
    }

    private static final class RendererImpl implements Renderer {
        volatile MeshFile mesh;
        volatile float yaw, pitch, scale = 1f;
        private int program;
        private int aPos, aNrm, uYaw, uPitch, uScale;
        private volatile boolean uploaded;
        private int nvert;
        private float fitScale = 1f;
        private FloatBuffer posBuf, nrmBuf;

        @Override
        public void onSurfaceCreated(GL10 gl, EGLConfig config) {
            GLES20.glClearColor(0.07f, 0.07f, 0.08f, 1f);
            GLES20.glEnable(GLES20.GL_DEPTH_TEST);
            String vs = "attribute vec3 aPos; attribute vec3 aNrm; uniform float uYaw; uniform float uPitch; uniform float uScale;"
                    + "varying float vLit;"
                    + "void main(){"
                    + "float cy=cos(uYaw), sy=sin(uYaw), cp=cos(uPitch), sp=sin(uPitch);"
                    + "vec3 p=aPos*uScale; vec3 n=aNrm;"
                    + "float x1=cy*p.x+sy*p.z; float z1=-sy*p.x+cy*p.z;"
                    + "float y2=cp*p.y-sp*z1; float z2=sp*p.y+cp*z1;"
                    + "float nx=cy*n.x+sy*n.z; float nz=-sy*n.x+cy*n.z; float ny=cp*n.y-sp*nz;"
                    + "vLit=clamp(0.35+0.65*dot(normalize(vec3(nx,ny,nz)), normalize(vec3(-0.3,0.6,0.8))), 0.2, 1.0);"
                    + "gl_Position=vec4(x1,y2,z2*0.15,1.0); gl_PointSize=6.0; }";
            String fs = "precision mediump float; varying float vLit; void main(){ gl_FragColor=vec4(vec3(vLit),1.0); }";
            program = link(vs, fs);
            uploaded = false;
            aPos = GLES20.glGetAttribLocation(program, "aPos");
            aNrm = GLES20.glGetAttribLocation(program, "aNrm");
            uYaw = GLES20.glGetUniformLocation(program, "uYaw");
            uPitch = GLES20.glGetUniformLocation(program, "uPitch");
            uScale = GLES20.glGetUniformLocation(program, "uScale");
        }

        @Override
        public void onSurfaceChanged(GL10 gl, int width, int height) {
            GLES20.glViewport(0, 0, width, height);
        }

        @Override
        public void onDrawFrame(GL10 gl) {
            GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);
            MeshFile m = mesh;
            if (m == null || m.xyz.length < 3) return;
            if (!uploaded) upload(m);
            if (program == 0 || posBuf == null) return;
            GLES20.glUseProgram(program);
            GLES20.glEnableVertexAttribArray(aPos);
            GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, 0, posBuf);
            GLES20.glEnableVertexAttribArray(aNrm);
            GLES20.glVertexAttribPointer(aNrm, 3, GLES20.GL_FLOAT, false, 0, nrmBuf);
            GLES20.glUniform1f(uYaw, yaw);
            GLES20.glUniform1f(uPitch, pitch);
            GLES20.glUniform1f(uScale, scale * fitScale);
            GLES20.glDrawArrays(m.pointsOnly ? GLES20.GL_POINTS : GLES20.GL_TRIANGLES, 0, nvert);
        }

        private void upload(MeshFile m) {
            float[] pos;
            float[] nrm;
            if (m.pointsOnly || m.idx.length < 3) {
                pos = Arrays.copyOf(m.xyz, m.xyz.length);
                nrm = m.nrm.length == m.xyz.length ? Arrays.copyOf(m.nrm, m.nrm.length) : new float[m.xyz.length];
                nvert = m.xyz.length / 3;
            } else {
                pos = new float[m.idx.length * 3];
                nrm = new float[m.idx.length * 3];
                for (int i = 0; i < m.idx.length; i++) {
                    int s = m.idx[i] * 3;
                    if (s < 0 || s + 2 >= m.xyz.length) continue;
                    pos[i * 3] = m.xyz[s];
                    pos[i * 3 + 1] = m.xyz[s + 1];
                    pos[i * 3 + 2] = m.xyz[s + 2];
                    if (s + 2 < m.nrm.length) {
                        nrm[i * 3] = m.nrm[s];
                        nrm[i * 3 + 1] = m.nrm[s + 1];
                        nrm[i * 3 + 2] = m.nrm[s + 2];
                    }
                }
                nvert = m.idx.length;
            }
            fitScale = fit(pos);
            posBuf = buffer(pos);
            nrmBuf = buffer(nrm);
            GLES20.glEnableVertexAttribArray(aPos);
            GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, 0, posBuf);
            GLES20.glEnableVertexAttribArray(aNrm);
            GLES20.glVertexAttribPointer(aNrm, 3, GLES20.GL_FLOAT, false, 0, nrmBuf);
            uploaded = true;
        }

        private static FloatBuffer buffer(float[] data) {
            FloatBuffer fb = ByteBuffer.allocateDirect(data.length * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
            fb.put(data).position(0);
            return fb;
        }

        private static float fit(float[] xyz) {
            float minx = 1e9f, miny = 1e9f, minz = 1e9f, maxx = -1e9f, maxy = -1e9f, maxz = -1e9f;
            for (int i = 0; i + 2 < xyz.length; i += 3) {
                minx = Math.min(minx, xyz[i]); maxx = Math.max(maxx, xyz[i]);
                miny = Math.min(miny, xyz[i + 1]); maxy = Math.max(maxy, xyz[i + 1]);
                minz = Math.min(minz, xyz[i + 2]); maxz = Math.max(maxz, xyz[i + 2]);
            }
            float cx = (minx + maxx) * 0.5f, cy = (miny + maxy) * 0.5f, cz = (minz + maxz) * 0.5f;
            for (int i = 0; i + 2 < xyz.length; i += 3) {
                xyz[i] -= cx; xyz[i + 1] -= cy; xyz[i + 2] -= cz;
            }
            float ext = Math.max(maxx - minx, Math.max(maxy - miny, maxz - minz));
            return ext < 1e-3f ? 1f : 1.4f / ext;
        }

        private static int link(String vs, String fs) {
            int v = shader(GLES20.GL_VERTEX_SHADER, vs);
            int f = shader(GLES20.GL_FRAGMENT_SHADER, fs);
            int p = GLES20.glCreateProgram();
            GLES20.glAttachShader(p, v);
            GLES20.glAttachShader(p, f);
            GLES20.glLinkProgram(p);
            return p;
        }

        private static int shader(int type, String src) {
            int s = GLES20.glCreateShader(type);
            GLES20.glShaderSource(s, src);
            GLES20.glCompileShader(s);
            int[] ok = new int[1];
            GLES20.glGetShaderiv(s, GLES20.GL_COMPILE_STATUS, ok, 0);
            if (ok[0] == 0) Log.e("openscan", GLES20.glGetShaderInfoLog(s));
            return s;
        }
    }
}
