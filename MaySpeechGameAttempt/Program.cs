using System;
using NAudio.Wave;
using System.Linq;
using System.Collections.Generic;

public class AudioDataEventArgs : EventArgs
{
    public float[] AudioData { get; set; }
}

// class to capture audio data from microphong using NAudio with an input of sample rate, channels, and frame length in samples


public class AudioCapture
{
    public event EventHandler<AudioDataEventArgs> DataAvailable;
    public WaveInEvent WaveIn { get; }

    public AudioCapture(int sampleRate = 44100, int channels = 1, int frameLength = 100)
    {
        WaveIn = new WaveInEvent
        {
            DeviceNumber = 0,
            WaveFormat = new WaveFormat(sampleRate, channels),
            BufferMilliseconds = frameLength
        };

        WaveIn.DataAvailable += OnDataAvailable;
    }

    public void Start() => WaveIn.StartRecording();

    public void Stop() => WaveIn.StopRecording();

    private void OnDataAvailable(object sender, WaveInEventArgs e)
    {
        var buffer = new float[e.BytesRecorded / 2];
        for (int i = 0; i < buffer.Length; i++)
        {
            buffer[i] = BitConverter.ToInt16(e.Buffer, i * 2) / 32768f; // convert to float
        }
        DataAvailable?.Invoke(this, new AudioDataEventArgs { AudioData = buffer });
    }
}



/*public class PitchDetector
{
    private int _sampleRate;
    private int _bufferSize;

    public PitchDetector(int sampleRate, int bufferSize)
    {
        _sampleRate = sampleRate;
        _bufferSize = bufferSize;
    }
    public double DetectPitch(float[] frame)
    {
        int maxLag = frame.Length;
        double[] autocorrelation = new double[maxLag + 1];
        for (int lag = 0; lag <= maxLag; lag++)
        {
            for (int i = 0; i < maxLag - lag; i++)
            {
                autocorrelation[lag] += frame[i] * frame[i + lag];
            }
        }

        // Initialize max and maxLagIndex to be -1
        double max = -1;
        int maxLagIndex = -1;

        // We ignore the lags below 882 since the minimum frequency is 50Hz
        int minLag = _sampleRate / 50;

        // First, find the maximum value for normalization
        double normalizationFactor = autocorrelation.Max();

        // Then, normalize the autocorrelation function
        for (int i = 0; i <= maxLag; i++)
        {
            autocorrelation[i] /= normalizationFactor;
        }

        // Ignore the peak at lag zero, and find the max peak for lags > minLag
        for (int lag = minLag; lag <= maxLag; lag++)
        {
            if (autocorrelation[lag] > max)
            {
                max = autocorrelation[lag];
                maxLagIndex = lag;
            }
        }

        // calculate pitch in hertz and return it
        double pitchInHertz;
        if (maxLagIndex != 0) // to avoid dividing by zero
        {
            pitchInHertz = _sampleRate / (double)maxLagIndex;
        }
        else
        {
            pitchInHertz = 0; // or whatever value you want to assign when pitch can't be detected
        }
        return pitchInHertz;
    }
}*/
public class PitchDetector
{
    private int _sampleRate;
    private int _bufferSize;

    public PitchDetector(int sampleRate, int bufferSize)
    {
        _sampleRate = sampleRate;
        _bufferSize = bufferSize;
    }

    public double DetectPitch(float[] buffer)
    {
        int halfBufferSize = _bufferSize / 2;
        double[] yinBuffer = new double[halfBufferSize];
        double runningSum = 0;

        for (int tau = 0; tau < halfBufferSize; tau++)
        {
            yinBuffer[tau] = 0;
        }

        for (int tau = 1; tau < halfBufferSize; tau++)
        {
            for (int i = 0; i < halfBufferSize; i++)
            {
                double delta = buffer[i] - buffer[i + tau];
                yinBuffer[tau] += delta * delta;
            }

            runningSum += yinBuffer[tau];

            if (tau != 0)
            {
                yinBuffer[tau] *= tau / runningSum;
            }
        }

        int tauEstimate = -1;
        for (int tau = 2; tau < halfBufferSize; tau++)
        {
            if (yinBuffer[tau] < 0.1 && yinBuffer[tau] < yinBuffer[tau - 1])
            {
                tauEstimate = tau;
                break;
            }
        }

        if (tauEstimate != -1)
        {
            double betterTau = tauEstimate;
            if (tauEstimate > 0 && tauEstimate < halfBufferSize - 1)
            {
                double s0, s1, s2;
                s0 = yinBuffer[tauEstimate - 1];
                s1 = yinBuffer[tauEstimate];
                s2 = yinBuffer[tauEstimate + 1];

                betterTau += 0.5 * (s2 - s0) / (2 * s1 - s0 - s2);
            }

            return _sampleRate / betterTau;
        }

        return 0;
    }
}

public class VolumeDetector
{
    private int _sampleRate;
    private int _bufferSize;

    public VolumeDetector(int sampleRate, int bufferSize)
    {
        _sampleRate = sampleRate;
        _bufferSize = bufferSize;
    }

    public double DetectVolume(float[] frame)
    {
        double sum = 0;
        for (int i = 0; i < frame.Length; i++)
        {
            sum += frame[i] * frame[i];  // square each sample
        }
        return Math.Sqrt(sum / frame.Length); // return the square root of the average
    }
}
/* //this class is used to create a sine wave to test the pitch detector
public class Program
{
    public static void Main(string[] args)
    {
        int sampleRate = 44100;
        int bufferSize = 8192;
        double frequency = 240.0; // A4 note
        double amplitude = 0.1;

        // Create a buffer and fill it with a sine wave
        float[] buffer = new float[bufferSize];
        for (int i = 0; i < bufferSize; i++)
        {
            buffer[i] = (float)(amplitude * Math.Sin((2 * Math.PI * i * frequency) / sampleRate));
        }

        // Create an instance of the PitchDetector
        PitchDetector detector = new PitchDetector(sampleRate, bufferSize);

        // Detect the pitch of the sine wave in the buffer
        double pitch = detector.DetectPitch(buffer);

        // Output the result
        Console.WriteLine($"Detected pitch: {pitch} Hz");
    }
}
*/

public class Program
{
    private static float[] frame1;
    private static float[] frame2;
    private static int frameIndex;
    private const int bufferSize = 8192;   // Define bufferSize
    private static VolumeDetector volumeDetector;


    // Instance of PitchTracker for pitch analysis
    static PitchDetector detector = new PitchDetector(44100, bufferSize);
    // NEW: Define the list to store pitch and volume values for the last 4 seconds
    static int frameHistorySize = 4 * 44100 / bufferSize; // NEW
    static List<double> pitchValues = new List<double>(frameHistorySize); // NEW
    static List<double> volumeValues = new List<double>(frameHistorySize); // NEW

    static void Main(string[] args)
    {
        const int sampleRate = 44100;
        const int frameLength = 250; // milliseconds

        volumeDetector = new VolumeDetector(sampleRate, bufferSize);

        var audioCapture = new AudioCapture(sampleRate, 1, frameLength);
        audioCapture.DataAvailable += OnDataAvailable;
        audioCapture.Start();
        Console.ReadKey();
        audioCapture.Stop();
    }

    static void OnDataAvailable(object sender, AudioDataEventArgs e)
    {
        // test code initializations

        int sampleRate = 44100;
        int bufferSize = 8192;
        double frequency = 244.0; // frequency in Hz
        double amplitude = 0.1;  // reasonable amplitude for microphone input// Switch between two frames (buffers)

        // Switch between two frames (buffers)
        float[] currentFrame;

        // end test code initializations


                if (frameIndex == 0)
                {
                    frame1 = e.AudioData;
                    currentFrame = frame1;
                    frameIndex = 1;
                }
                else
                {
                    frame2 = e.AudioData;
                    currentFrame = frame2;
                    frameIndex = 0;
                }
               
        // Compute pitch of current frame using DetectPitch method
        double pitchValue = detector.DetectPitch(currentFrame);
        double volumeValue = volumeDetector.DetectVolume(currentFrame);

        // NEW: Remove the oldest value if the history size limit is reached
        if (pitchValues.Count >= frameHistorySize) // NEW
        {
            pitchValues.RemoveAt(0); // NEW
            volumeValues.RemoveAt(0); // NEW
        }

        // NEW: Add the new values
        pitchValues.Add(pitchValue); // NEW
        volumeValues.Add(volumeValue); // NEW


        Console.WriteLine($"Frame Filled - frame index: {frameIndex}, Pitch: {pitchValue} Hz, Volume: {volumeValue}");
    }

}









/*
using System;
using System.Linq;
using Microsoft.VisualBasic;
using NAudio.Wave;
using System.Collections.Generic;

public class Program
{
    static void Main()
    {
        const int frameLength = 100; // in milliseconds
        var audioCapture = new AudioCapture(frameLength: frameLength);
        var pitchAnalyzer = new PitchAnalyzer();

        audioCapture.DataAvailable += (sender, e) =>
        {
            pitchAnalyzer.AnalyzePitch(e.AudioData, audioCapture.WaveIn.WaveFormat.SampleRate);
            var lastPitch = pitchAnalyzer.GetLastFramePitches().LastOrDefault();
            Console.WriteLine($"Frame filled. Detected pitch: {lastPitch} Hz");
        };

        audioCapture.Start();

        // Add code to stop recording or application will end immediately
    }
}
public class DataAvailableEventArgs : EventArgs
{
    public float[] AudioData { get; set; }
}
class AudioCapture
{
    private float[][] buffers = new float[2][];
    private int currentBuffer = 0;

    private WaveInEvent waveIn;

    public int FrameLength { get; }

    public event EventHandler<DataAvailableEventArgs> DataAvailable;

    public AudioCapture(int frameLength)
    {
        FrameLength = frameLength;

        // Initialize NAudio wave input
        waveIn = new WaveInEvent
        {
            BufferMilliseconds = frameLength,
            NumberOfBuffers = 2,
            WaveFormat = new WaveFormat(44100, 1) // Mono PCM 44.1 kHz
        };

        waveIn.DataAvailable += (s, e) =>
        {
            // Convert the byte buffer to float (mono)
            float[] floatBuffer = ConvertToMono(e.Buffer, e.BytesRecorded);

            // Fill the current buffer
            buffers[currentBuffer] = floatBuffer;
            // Switch to the other buffer
            currentBuffer = (currentBuffer + 1) % 2;

            DataAvailable?.Invoke(this, new DataAvailableEventArgs { AudioData = floatBuffer });
        };
    }

    // Conversion from byte array to mono (float array)
    private float[] ConvertToMono(byte[] audioBytes, int bytesRecorded)
    {
        int sampleCount = bytesRecorded / 2; // 2 bytes per sample (16 bit)
        float[] mono = new float[sampleCount];
        for (int i = 0; i < sampleCount; i++)
        {
            mono[i] = BitConverter.ToInt16(audioBytes, i * 2) / 32768f; // normalize to [-1, 1]
        }
        return mono;
    }

    public float[] GetCurrentBuffer()
    {
        return buffers[currentBuffer];
    }

    public float[] GetPreviousBuffer()
    {
        return buffers[(currentBuffer + 1) % 2];
    }

    public void StartCapturing()
    {
        waveIn.StartRecording();
    }

    public void StopCapturing()
    {
        waveIn.StopRecording();
    }
}
public class PitchAnalyzer
{
    public Queue<float> PitchQueue { get; }
    public int FramePitchLength { get; }

    public PitchAnalyzer(int framePitchLength = 80)
    {
        FramePitchLength = framePitchLength;
        PitchQueue = new Queue<float>(framePitchLength);
    }

    public void AnalyzePitch(float[] buffer, int sampleRate)
    {
        var pitchDetector = new PitchDetector(sampleRate, buffer.Length);
        var pitch = pitchDetector.DetectPitch(buffer, buffer.Length);
        if (PitchQueue.Count >= FramePitchLength)
        {
            PitchQueue.Dequeue();
        }
        PitchQueue.Enqueue(pitch);
    }

    public float[] GetLastFramePitches()
    {
        return PitchQueue.ToArray();
    }
}
public class PitchDetector
{
    private int _sampleRate;
    private int _bufferSize;

    public PitchDetector(int sampleRate, int bufferSize)
    {
        _sampleRate = sampleRate;
        _bufferSize = bufferSize;
    }

    public float DetectPitch(float[] buffer, int frames)
    {
        int maxShift = frames;

        float[] autoc = new float[maxShift + 1];

        // Autocorrelation.
        for (int shift = 0; shift <= maxShift; shift++)
        {
            float sum = 0;
            for (int i = 0; i < frames - shift; i++)
            {
                sum += (buffer[i] * buffer[i + shift]);
            }
            autoc[shift] = sum;
        }

        // Find the first minimum
        int minIndex = 0;
        while ((minIndex < maxShift) && (autoc[minIndex] > autoc[minIndex + 1]))
        {
            minIndex++;
        }

        // Find the next peak
        int maxIndex = 0;
        float maxValue = 0.0f;
        for (int i = minIndex; i < maxShift; i++)
        {
            if (autoc[i] > maxValue)
            {
                maxValue = autoc[i];
                maxIndex = i;
            }
        }

        float pitch;
        if (maxIndex == 0)
            pitch = -1;
        else
            pitch = (float)_sampleRate / maxIndex;

        return pitch;
    }
}
public class AudioDataEventArgs : EventArgs
{
    public float[] AudioData { get; set; }
}
public class AudioCapture
{
    public event EventHandler<AudioDataEventArgs> DataAvailable;
    public WaveInEvent WaveIn { get; }

    public AudioCapture(int sampleRate = 44100, int channels = 1, int frameLength = 100)
    {
        WaveIn = new WaveInEvent
        {
            DeviceNumber = 0,
            WaveFormat = new WaveFormat(sampleRate, channels),
            BufferMilliseconds = frameLength
        };

        WaveIn.DataAvailable += OnDataAvailable;
    }

    public void Start() => WaveIn.StartRecording();

    public void Stop() => WaveIn.StopRecording();

    private void OnDataAvailable(object sender, WaveInEventArgs e)
    {
        var buffer = new float[e.BytesRecorded / 2];
        for (int i = 0; i < buffer.Length; i++)
        {
            buffer[i] = BitConverter.ToInt16(e.Buffer, i * 2) / 32768f; // convert to float
        }
        DataAvailable?.Invoke(this, new AudioDataEventArgs { AudioData = buffer });
    }
}
*/