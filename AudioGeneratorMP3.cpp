/*
  AudioGeneratorMP3
  Wrap libmad MP3 library to play audio
  
  Copyright (C) 2017  Earle F. Philhower, III

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "AudioGeneratorMP3.h"

AudioGeneratorMP3::AudioGeneratorMP3()
{
  running = false;
  file = NULL;
  output = NULL;
  buffLen = 2048;
  buff = NULL;
}

AudioGeneratorMP3::~AudioGeneratorMP3()
{
  free(buff);
  buff = NULL;
}

bool AudioGeneratorMP3::SetBufferSize(int sz)
{
  if (running) return false;
  buffLen = sz;
  return true;
}


bool AudioGeneratorMP3::stop()
{
  if (!running) return true;
  running = false;
  free(buff);
  buff = NULL;
  
  mad_synth_finish(&synth);
  mad_frame_finish(&frame);
  mad_stream_finish(&stream);

  return file->close();
}

bool AudioGeneratorMP3::isRunning()
{
  return running;
}

enum mad_flow AudioGeneratorMP3::ErrorToFlow()
{
  char err[64];
  strcpy_P(err, mad_stream_errorstr(&stream));
  Serial.printf("Decoding error 0x%04x (%s) at byte offset %d\n", stream.error, err, (stream.this_frame - buff) + lastReadPos);
  Serial.flush();
  return MAD_FLOW_CONTINUE;
}

enum mad_flow AudioGeneratorMP3::Input()
{
  int unused = 0;
  if (stream.next_frame) {
    unused = buffLen - (stream.next_frame - buff);
    memmove(buff, stream.next_frame, unused);
  }
  if (unused == buffLen) 
    return MAD_FLOW_STOP;

  lastReadPos = file->getPos() - unused;
  int len = buffLen - unused;
  len = file->read(buff + unused, len);
  if (len == 0) return MAD_FLOW_STOP;

  mad_stream_buffer(&stream, buff, len + unused);

  return MAD_FLOW_CONTINUE;
}


bool AudioGeneratorMP3::DecodeNextFrame()
{
  do {
    if (!inInnerDecode) {
      if (Input() == MAD_FLOW_STOP) return false;
    }
    while (1) {
      inInnerDecode = true;
      if (mad_frame_decode(&frame, &stream) == -1) {
        if (!MAD_RECOVERABLE(stream.error)) break;
        ErrorToFlow(); // Always returns CONTINUE
        continue;
      }
      return true;
    }
    inInnerDecode = false;
  } while (stream.error == MAD_ERROR_BUFLEN);

  return false;
}

bool AudioGeneratorMP3::GetOneSample(int16_t sample[2])
{
  if ( (samplePtr >= synth.pcm.length) && (nsCount >= 1152/32) ) {
    if (!DecodeNextFrame()) return false;
    samplePtr = 9999;
    nsCount = 0;
  }

  if (synth.pcm.samplerate != lastRate) {
    output->SetRate(synth.pcm.samplerate);
    lastRate = synth.pcm.samplerate;
  }
  if (synth.pcm.channels != lastChannels) {
    output->SetChannels(synth.pcm.channels);
    lastChannels = synth.pcm.channels;
  }
    
  // If we're here, we have one decoded frame and sent 0 or more samples out
  if (samplePtr < synth.pcm.length) {
    sample[AudioOutput::LEFTCHANNEL ] = synth.pcm.samples[0][samplePtr];
    sample[AudioOutput::RIGHTCHANNEL] = synth.pcm.samples[1][samplePtr];
    samplePtr++;
  } else {
    samplePtr = 0;
    
    switch ( mad_synth_frame_onens(&synth, &frame, nsCount++) ) {
        case MAD_FLOW_STOP:
        case MAD_FLOW_BREAK:
          return false; // Either way we're done
        default:
          break; // Do nothing
    }
    // for IGNORE and CONTINUE, just play what we have now
    sample[AudioOutput::LEFTCHANNEL ] = synth.pcm.samples[0][samplePtr];
    sample[AudioOutput::RIGHTCHANNEL] = synth.pcm.samples[1][samplePtr];
    samplePtr++;
  }
  return true;
}


bool AudioGeneratorMP3::loop()
{
  static int16_t lastSample[2]= {0,0};
  
  if (!running) return true; // Nothing to do here!

  // First, try and push in the stored sample.  If we can't, then punt and try later
  if (!output->ConsumeSample(lastSample)) return true; // Can't send, but no error detected
  // Try and stuff the buffer one sample at a time
  do
  {
    if (!GetOneSample(lastSample)) {
      running = false;
      return false;
    }
  } while (running && output->ConsumeSample(lastSample));

  return running;
}



bool AudioGeneratorMP3::begin(AudioFileSource *source, AudioOutput *output)
{
  if (!source) return false;
  file = source;
  if (!output) return false;
  this->output = output;
  if (!file->isOpen()) return false; // Error
  if (!output->begin()) return false;

  // Where we are in generating one frame's data, set to invalid so we will run loop on first getsample()
  samplePtr = 9999;
  nsCount = 9999;
  synth.pcm.length = 0;
  lastRate = 0;
  lastChannels = 0;
  lastReadPos = 0;
  inInnerDecode = false;
  buff = reinterpret_cast<unsigned char *>(malloc(buffLen));
  if (!buff) return false;
  
  mad_stream_init(&stream);
  mad_frame_init(&frame);
  mad_synth_init(&synth);

  mad_stream_options(&stream, 0); // TODO - add options suppoirt
 
  running = true;
  return true;
}

