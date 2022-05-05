#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

void processAudioData(int16_t *data, uint16_t num_samples);

float get_angle(void);

void audio_init(void);

#endif /* AUDIO_PROCESSING_H */
