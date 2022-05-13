#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#define FFT_SIZE 	1024

void audio_init(void);

void processAudioData(int16_t *data, uint16_t num_samples);

float get_angle(void);
int16_t get_freq_index(void);

#endif /* AUDIO_PROCESSING_H */
