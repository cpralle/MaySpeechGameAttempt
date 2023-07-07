using System;
using NAudio.Wave;
using System.Linq;
using System.Collections.Generic;
using Microsoft.CognitiveServices.Speech;
using System.Diagnostics;

public class AudioDataEventArgs : EventArgs
{
    public float[] AudioData { get; set; }
}

// class to capture audio data from microphong using NAudio with an input of sample rate, channels, and frame length in samples


public class AudioCapture
{
    public event EventHandler<AudioDataEventArgs> DataAvailable;
    public WaveInEvent WaveIn { get; }

    public AudioCapture(int sampleRate = 44100, int channels = 1, int frameLength = 250)  // TRYING frameLength = 250, was 100
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
        double threshold = 0.4;  

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
            if (yinBuffer[tau] < threshold && yinBuffer[tau] < yinBuffer[tau - 1])  
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


public class Program
{
    private static float[] frame1;
    private static float[] frame2;
    private static int frameIndex;
    private const int bufferSize = 11024;   // Define bufferSize, changing to 11024 from 8192 to match a 250ms frame length
    private static VolumeDetector volumeDetector;

    // Azure Cognitive Services setup
    private static string azureKey = "0153c9f157904ca180837190d0ca73df"; 
    private static string azureRegion = "westus"; 
    private static SpeechRecognizer recognizer; 

    // Instance of PitchTracker for pitch analysis
    static PitchDetector detector = new PitchDetector(44100, bufferSize);
    // Define the list to store pitch and volume values for the last 4 seconds
    static int frameHistorySize = 4 * 44100 / bufferSize; 
    static List<double> pitchValues = new List<double>(frameHistorySize); 
    static List<double> volumeValues = new List<double>(frameHistorySize); 

    private static Stopwatch phraseStopwatch = new Stopwatch(); 
    private static double minPitchValue = 200.0; 

    static async Task Main(string[] args) 
    {
        const int sampleRate = 44100;
        const int frameLength = 250; // milliseconds

        volumeDetector = new VolumeDetector(sampleRate, bufferSize);

        // Setup and start Azure Cognitive Services Speech Recognition
        var config = SpeechConfig.FromSubscription(azureKey, azureRegion); 
        config.SetProfanity(ProfanityOption.Raw);
        recognizer = new SpeechRecognizer(config);
        recognizer.Recognized += Recognizer_Recognized; 
        recognizer.Recognizing += Recognizer_Recognizing; 
        recognizer.Canceled += Recognizer_Canceled; 
        await recognizer.StartContinuousRecognitionAsync(); 

        var audioCapture = new AudioCapture(sampleRate, 1, frameLength);
        audioCapture.DataAvailable += OnDataAvailable;
        audioCapture.Start();
        Console.ReadKey();
        audioCapture.Stop();

        // Stop Azure Cognitive Services Speech Recognition
        await recognizer.StopContinuousRecognitionAsync(); 
    }
    private static void Recognizer_Recognizing(object sender, SpeechRecognitionEventArgs e) 
    {
        // Start stopwatch when a phrase starts
        if (!phraseStopwatch.IsRunning)
        {
            phraseStopwatch.Start();
        }
    }
    private static void Recognizer_Recognized(object sender, SpeechRecognitionEventArgs e) 
    {
        // When a phrase ends
        if (!string.IsNullOrEmpty(e.Result.Text))
        {
            
            // Calculate the length of the phrase in seconds
            double phraseLength = phraseStopwatch.Elapsed.TotalSeconds;
            phraseStopwatch.Reset();

            Console.WriteLine($"Azure Speech Recognition: {e.Result.Text}, Duration:  {phraseLength} s");

            // Determine the number of frames to check
            int framesToCheck = (int)(phraseLength * 44100 / bufferSize) + 8;

            // Initialize sum and count for average calculation
            double pitchSum = 0;
            int validPitchCount = 0;
            
            // Check the corresponding pitch values
            for (int i = Math.Max(0, pitchValues.Count - framesToCheck); i < pitchValues.Count; i++)
            {
                double pitch = pitchValues[i];

                // If the pitch value is 0, negative, or above 600, ignore it and continue to the next iteration
                if (pitch <= 0 || pitch > 600)
                {
                    continue;
                }

                // Add the valid pitch value to the sum and increase the valid pitch count
                pitchSum += pitch;
                validPitchCount++;
            }
            // Calculate and output the average pitch value, if any valid pitches were found
            if (validPitchCount > 0)
            {
                double averagePitch = pitchSum / validPitchCount;
                Console.WriteLine($"Average Valid Pitch: {averagePitch} Hz");

                // If the average pitch is less than minPitchValue, output PITCH FAIL
                if (averagePitch < minPitchValue)
                {
                    Console.WriteLine($"PITCH FAIL, Average Valid Pitch:  {averagePitch} Hz");
                }
            }
        }
    }
    private static void Recognizer_Canceled(object sender, SpeechRecognitionCanceledEventArgs e) 
    {
        // When speech recognition is canceled, reset the stopwatch
        phraseStopwatch.Reset();
    }

    // A variable to count the number of silent frames since the last non-silent frame
    private static int silentFrameCount = 0;
    private static int maxSilentFrames = 32; // Adjust this value as needed
    private static float silentFrameThreshold = 0.01f; // Adjust this value as needed

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

        // Remove the oldest value if the history size limit is reached
        if (pitchValues.Count >= frameHistorySize) 
        {
            pitchValues.RemoveAt(0); 
            volumeValues.RemoveAt(0); 
        }

        // Add the new values
        pitchValues.Add(pitchValue); 
        volumeValues.Add(volumeValue);

        // Check if the frame is silent
        if (volumeValue <= silentFrameThreshold)
        {
            silentFrameCount++;

            if (silentFrameCount >= maxSilentFrames)
            {
                Console.WriteLine("SILENCE FAIL");
                silentFrameCount = 0; // reset the counter
            }
        }
        else
        {
            silentFrameCount = 0; // reset the counter
        }
        //Console.WriteLine($"Frame Filled - frame index: {frameIndex}, Pitch: {pitchValue} Hz, Volume: {volumeValue}");
    }

}
