package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.GlobalVariables;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.IOException;
import java.util.Vector;
import java.util.HashMap;
import java.sql.Timestamp;

import java.net.*;
import java.io.*;
import java.util.*;

public class TelemetryEx {
    private boolean m_loggingEnabled = true;
    private static final int port = 9000;
    private long m_epic;
    private Telemetry m_telemetry;
    public final int m_queueSize = 20000; //1000; //100000;
    public volatile int m_headQueueIndex = 0;
    private volatile boolean m_restarted = true;
    Vector<LogEntry> m_log = new Vector<LogEntry>();
    private HashMap<String, String> m_lastValues = new HashMap<String, String>();
    
    private Thread httpServerThread;

    private class LogEntry {
        private long m_timeStamp;
        private String m_caption;
        private String m_value;
        
        public LogEntry(String caption, String value) {
            m_caption = caption;
            m_value = value;
            if (m_epic == -1)
                m_epic = System.currentTimeMillis();
            m_timeStamp = System.currentTimeMillis() - m_epic;
        }
        
        public long timeStamp() {
            return m_timeStamp;
        }
        
        public String caption() {
            return m_caption;
        }
        
        public String value() {
            return m_value;
        }

    };

    public TelemetryEx() {
    }

    private static TelemetryEx m_Instance = null;
    public synchronized static TelemetryEx getInstance() {
        if (m_Instance == null) {
            m_Instance = new TelemetryEx();
        }
        return m_Instance;
    }

    public String encode(String s) {
        String enc = s.replace("\\", "\\\\");
        return enc.replace("\"", "\\\"");
    }

    public String decode(String s) {
        String dec = s.replace("\\\"", "\"");
        return dec.replace("\\\\", "\\");
    }

    private void stopOldServer(int port, int pause) {
        HttpURLConnection urlConnection = null;
        try {
            URL url = new URL(String.format("http://localhost:%d/stop", port));
            urlConnection = (HttpURLConnection) url.openConnection();
            urlConnection.getInputStream();
        } catch (Exception ex) {
        }
        if (urlConnection != null)
            urlConnection.disconnect();
        try {
            Thread.sleep(pause);
        } catch (InterruptedException ex) {
        }
    }

    private class HttpServerThread extends Thread {
        static final String newLine = "\r\n";
        
        public HttpServerThread() {
            this.setName("HttpServer Thread");
        }

        @Override
        public void run() {
            try {
                stopOldServer(port, 750);
                ServerSocket socket = new ServerSocket(port);

                while (true) {
                    Socket connection = socket.accept();

                    try {
                        BufferedReader in = new BufferedReader(new InputStreamReader(connection.getInputStream()));
                        OutputStream out = new BufferedOutputStream(connection.getOutputStream());
                        PrintStream pout = new PrintStream(out);

                        // read first line of request
                        String request = in.readLine();
                        if (request == null) {
                            connection.close();
                            continue;
                        }

                        // we ignore the rest
                        while (true) {
                            String ignore = in.readLine();
                            if (ignore == null || ignore.length() == 0) break;
                        }

                        if (!request.startsWith("GET ") ||
                            !(request.endsWith(" HTTP/1.0") || request.endsWith(" HTTP/1.1")))
                        {
                            // bad request
                            pout.print("HTTP/1.0 400 Bad Request" + newLine + newLine);
                        }
                        else {
                            String path = request.substring(4, request.length() - 9);

                            int head = m_headQueueIndex;
                            int index = 0;
                            String[] parms = path.split("/", -1);
                            if (parms.length >= 2 && parms[1].equals("get")) {
                                if (!m_restarted && parms.length == 3)
                                    index = Integer.parseInt(parms[2]);
                                else if (m_log.size() == m_queueSize)
                                    index = head;
    
                                String log = "";
                                int start = Math.min(index, m_queueSize);
                                int stop = Math.min(m_queueSize, m_log.size());
                                stop = Math.min(stop, head);
                                int count = start <= stop ? stop - start : m_queueSize - start + stop;
                                for (int i = 0; i < count; i++) {
                                    LogEntry entry = m_log.get(start);
                                    log += String.format("%d %.3f \"%s\" \"%s\"\n", m_restarted ? -1 : start,
                                                         entry.timeStamp() / 1000.0,
                                                         encode(entry.caption()), encode(entry.value()));
                                    start++;
                                    if (start == m_queueSize)
                                        start = 0;
                                }

                                pout.print(
                                    "HTTP/1.0 200 OK" + newLine +
                                    "Content-Type: text/plain" + newLine +
                                    "Date: " + new Date() + newLine +
                                    "Content-length: " + log.length() + newLine + newLine +
                                    log
                                );
                            }
                            else if (parms.length >= 2 && parms[1].equals("stop")) {
                                String msg = "Stopping";
                                pout.print(
                                    "HTTP/1.0 200 OK" + newLine +
                                    "Content-Type: text/plain" + newLine +
                                    "Date: " + new Date() + newLine +
                                    "Content-length: " + msg.length() + newLine + newLine +
                                    msg
                                );
                                connection.close();
                                break;
                            }
                            else {
                                String msg = "Unknown request: " + path;
                                pout.print(
                                    "HTTP/1.0 200 OK" + newLine +
                                    "Content-Type: text/plain" + newLine +
                                    "Date: " + new Date() + newLine +
                                    "Content-length: " + msg.length() + newLine + newLine +
                                    msg
                                );
                            }

                            m_restarted = false;
                        }

                        pout.close();
                    }
                    catch (Throwable tri) {
                        // System.err.println("Error handling request: "+tri);
                    }
                    connection.close();
                }
                socket.close();
            } catch (Exception e) {} ;
        } 
    }


    public void init(Telemetry telemetry, boolean enabled) {
        m_restarted = true;
        m_telemetry = telemetry;
        m_loggingEnabled = enabled;
        m_log.clear();
        m_epic = -1;
        m_lastValues.clear();
        m_headQueueIndex = 0;

        if (m_loggingEnabled) {
            httpServerThread = new HttpServerThread();
            httpServerThread.start();
        }
        else {
            stopOldServer(port, 0);
        }
    }

    public void addToLog(String caption, String value) {
        if (m_lastValues.containsKey(caption) && value.equals(m_lastValues.get(caption)))
            return;

        m_lastValues.put(caption, value);
            
        if (m_headQueueIndex < m_log.size())
            m_log.set(m_headQueueIndex, new LogEntry(caption, value));
        else
            m_log.add(new LogEntry(caption, value));
        // Move to the next slot in the queue and wrap around to the beginning if necessary.
        m_headQueueIndex++;
        if (m_headQueueIndex == m_queueSize)
            m_headQueueIndex = 0;
    }

    // public Telemetry.Item addData(java.lang.String caption, java.lang.String format, java.lang.Object... args) {
    public void addData(java.lang.String caption, java.lang.String format, java.lang.Object... args) {
        if (m_loggingEnabled)
            addToLog(caption, String.format(format, args));
        m_telemetry.addData(caption, format, args);
    }

    public void addData(java.lang.String caption, java.lang.Object value) {
        if (m_loggingEnabled)
            addToLog(caption, value.toString());
        m_telemetry.addData(caption, value);
    }

    // NOTE: The OpMode telemetry.update() routine might get called without this update
    // being called.
    public boolean update() {
        return m_telemetry.update();
    }
}
