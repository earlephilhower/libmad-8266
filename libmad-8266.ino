#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "mad.h"
#include "AudioFileSourceSPIFFS.h"
#include "AudioOutputI2SDAC.h"

extern "C" {
  #include <cont.h>
  extern cont_t g_cont;

  void stack(char *s, char *t, int i) {
    register uint32_t *sp asm("a1");
    int freestack = 4 * (sp - g_cont.stack);
    int freeheap = ESP.getFreeHeap();
    static int laststack, lastheap;
//    if (laststack!=freestack|| lastheap !=freeheap)
//      Serial.printf("%s: FREESTACK=%d, FREEHEAP=%d\n", s, /*t, i,*/ freestack, /*cont_get_free_stack(&g_cont),*/ freeheap); Serial.flush();
    if (freestack < 256) {Serial.printf("out of stack!\n"); while (1); }
    if (freeheap < 1024) {Serial.printf("out of heap!\n"); while (1); }
    laststack=freestack;lastheap=freeheap;
  }
}
/*
 * This is a private message structure. A generic pointer to this structure
 * is passed to each of the callback functions. Put here any data you need
 * to access from within the callbacks.
 */

class Buffer
{
  public:
    Buffer() { lastRate=0; lastChannels=0; lastReadPos = 0;}
    unsigned char space[2048];
    AudioFileSourceSPIFFS file;
    AudioOutputI2SDAC i2sdac;
    int lastRate;
    int lastChannels;
    int lastReadPos;
};

/*
 * This is the input callback. The purpose of this callback is to (re)fill
 * the stream buffer which is to be decoded. In this example, an entire file
 * has been mapped into memory, so we just call mad_stream_buffer() with the
 * address and length of the mapping. When this callback is called a second
 * time, we are finished decoding.
 */
static enum mad_flow input(void *data,
                    struct mad_stream *stream)
{
  Buffer *buffer = reinterpret_cast<Buffer*>(data);

  int unused = 0;
  if (stream->next_frame) {
    unused = sizeof(buffer->space) - (stream->next_frame - buffer->space);
    memmove(buffer->space, stream->next_frame, unused);
  }
  if (unused == sizeof(buffer->space)) 
    return MAD_FLOW_STOP;

  buffer->lastReadPos = buffer->file.getPos() - unused;
  int len = sizeof(buffer->space) - unused;
  len = buffer->file.read(buffer->space + unused, len);
  if (len == 0) return MAD_FLOW_STOP;
  
  mad_stream_buffer(stream, buffer->space, len + unused);
  
  return MAD_FLOW_CONTINUE;
}

/*
 * The following utility routine performs simple rounding, clipping, and
 * scaling of MAD's high-resolution samples down to 16 bits. It does not
 * perform any dithering or noise shaping, which would be recommended to
 * obtain any exceptional audio quality. It is therefore not recommended to
 * use this routine if high-quality output is desired.
 */


/*
 * This is the output callback function. It is called after each frame of
 * MPEG audio data has been completely decoded. The purpose of this callback
 * is to output (or play) the decoded PCM audio.
 */

static enum mad_flow output(void *data,
                     struct mad_header const *header,
                     struct mad_pcm *pcm)
{
  unsigned int nchannels, nsamples;
  int16_t const *left_ch, *right_ch;
  Buffer *buffer = reinterpret_cast<Buffer *>(data);

  /* pcm->samplerate contains the sampling frequency */
  if (pcm->samplerate != buffer->lastRate) {
    buffer->i2sdac.SetRate(pcm->samplerate);
    buffer->lastRate = pcm->samplerate;
    Serial.printf("Setting sample rate: %d\n", pcm->samplerate);
  }
  if (pcm->channels != buffer->lastChannels) {
    buffer->i2sdac.SetChannels(pcm->channels);
    buffer->lastChannels = pcm->channels;
    Serial.printf("Setting channels: %d\n", pcm->channels);
  }
  nchannels = pcm->channels;
  nsamples  = pcm->length;
  left_ch   = pcm->samples[0];
  right_ch  = pcm->samples[1];

  while (nsamples--) {
    int16_t sample[2];
    sample[AudioOutput::LEFTCHANNEL] = *(left_ch++);
    sample[AudioOutput::RIGHTCHANNEL] = *(right_ch++);
    while (!buffer->i2sdac.ConsumeSample(sample)) {yield();};
  }

  return MAD_FLOW_CONTINUE;
}

/*
 * This is the error callback function. It is called whenever a decoding
 * error occurs. The error is indicated by stream->error; the list of
 * possible MAD_ERROR_* errors can be found in the mad.h (or stream.h)
 * header file.
 */

static enum mad_flow error(void *data,
                    struct mad_stream *stream,
                    struct mad_frame *frame)
{
  Buffer *buffer = reinterpret_cast<Buffer *>(data);
  
  char err[64];
  strcpy_P(err, mad_stream_errorstr(stream));
  Serial.printf("Decoding error 0x%04x (%s) at byte offset %p\n", stream->error, err, (stream->this_frame - buffer->space) + buffer->lastReadPos);
  Serial.flush();
  /* return MAD_FLOW_BREAK here to stop decoding (and propagate an error) */

  return MAD_FLOW_CONTINUE;
}

/*
 * This is the function called by main() above to perform all the decoding.
 * It instantiates a decoder object and configures it with the input,
 * output, and error callback functions above. A single call to
 * mad_decoder_run() continues until a callback function returns
 * MAD_FLOW_STOP (to stop decoding) or MAD_FLOW_BREAK (to stop decoding and
 * signal an error).
 */

static int decode()
{
  Buffer *buffer = new Buffer;
  struct mad_decoder decoder;
  int result;

  /* initialize our private message structure */
  buffer->file.open("/jamonit.mp3");
  buffer->i2sdac.begin();
  buffer->i2sdac.SetBitsPerSample(16);

  /* configure input, output, and error functions */

  mad_decoder_init(&decoder, buffer,
                   input, 0 /* header */, 0 /* filter */, output,
                   error, 0 /* message */);

  /* start decoding */

  result = mad_decoder_run(&decoder, MAD_DECODER_MODE_SYNC);

  /* release the decoder */

  mad_decoder_finish(&decoder);

  return result;
}


void setup()
{
  WiFi.forceSleepBegin();
  Serial.begin(115200);
  delay(1000);
  Serial.println("Begin recording...");
  delay(1000);
  decode();
}

void loop()
{
  Serial.println("loop()");
  delay(1000);  
}

