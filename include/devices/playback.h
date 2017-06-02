#ifndef LIBCAER_HDR_PLAYBACK_H_
#define LIBCAER_HDR_PLAYBACK_H_

#ifdef __cplusplus
extern "C" {
#endif
    #include "davis.h"

    typedef struct playback_state *playbackState;
    typedef struct playback_handle *playbackHandle;

    playbackHandle playbackOpen( const char *fileName);
    int playbackClose(playbackHandle handle);

    int playbackDataStart(playbackHandle handle);
    int playbackDataStop(playbackHandle handle);
    caerEventPacketContainer playbackDataGet(playbackHandle handle);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_HDR_PLAYBACK_H_ */
