# Mel-Spectrogram in Python

```python
import librosa

# default loading frequence is 16k.
def mel_spectrogram(filename, sr=16000, frame_length=0.025, frame_stride=0.010)

	# load wave from file
	y, sr = librosa.load(filename, sr=sr)

    # wave_seconds = len(y)/sr
	input_nfft = int(round(sr*frame_length))
	input_stride = int(round(sr*frame_stride))
    
    S = librosa.feature.melspectrogram(y=y, n_mels=40, n_fft=input_nfft, hop_length=input_stride)
```

