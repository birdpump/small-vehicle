import React, {useEffect, useRef, useState} from 'react';
import ReactPlayer from 'react-player'
import {PulseLoader, ScaleLoader} from "react-spinners";
import '../rtc.css';

const WebRTCComponent = () => {
    const [stream, setStream] = useState(null);
    const [fps, setFPS] = useState(0);
    const [jitter, setJitter] = useState(0);
    const [bitrate, setBitrate] = useState(0);
    const [isLoading, setIsLoading] = useState(true);
    const [isDisconnected, setIsDisconnected] = useState(false);
    const [inErrorState, setErrorState] = useState(false);
    const [errorMessage, setErrorMessage] = useState('');
    let [isInitialized, setIsInitialized] = useState(false);
    const [loading, setLoading] = useState(false);

    const pcRef = useRef(null);

    useEffect(() => {
        if (!isInitialized) {
            setIsInitialized(true);
            setTimeout(() => {
                start();
            }, 2000);
        }
    }, [isInitialized]);

    function negotiate() {
        console.log('Starting negotiation');
        pcRef.current.addTransceiver('video', {direction: 'recvonly'});
        return pcRef.current.createOffer().then((offer) => {
            return pcRef.current.setLocalDescription(offer);
        }).then(() => {
            return new Promise((resolve) => {
                if (pcRef.current.iceGatheringState === 'complete') {
                    resolve();
                } else {
                    const checkState = () => {
                        if (pcRef.current.iceGatheringState === 'complete') {
                            pcRef.current.removeEventListener('icegatheringstatechange', checkState);
                            resolve();
                        }
                    };
                    pcRef.current.addEventListener('icegatheringstatechange', checkState);
                }
            });
        }).then(() => {
            console.log("Requesting offer")
            const offer = pcRef.current.localDescription;
            return fetch('http://10.101.145.233:8080/offer', { //change this to the actual server later
                body: JSON.stringify({
                    sdp: offer.sdp,
                    type: offer.type,
                }),
                mode: 'cors',
                headers: {
                    'Content-Type': 'application/json'
                },
                method: 'POST',
            });
        }).then((response) => {
            return response.json();
        }).then((answer) => {
            console.log("Received offer data");
            return pcRef.current.setRemoteDescription(answer);
        }).catch((e) => {
            console.log(e);
            setErrorMessage('WebRTC Offer Request Failed');
            setErrorState(true);
        });
    }

    async function start() {
        setErrorState(false);
        setIsLoading(true);

        const config = {
            sdpSemantics: 'unified-plan'
        };

        pcRef.current = new RTCPeerConnection(config);
        // pcRef.current.setJitterBufferMaxPackets(100);

        pcRef.current.addEventListener('track', (evt) => {


            console.log("Starting stream");
            setIsLoading(false);
            setStream(evt.streams[0]);
        });

        pcRef.current.addEventListener('iceconnectionstatechange', () => {
            const connectionState = pcRef.current.iceConnectionState;
            console.log('ICE connection state changed to:', connectionState);

            // Handle disconnection based on ICE connection state
            if (connectionState === 'disconnected' || connectionState === 'failed') {
                setIsDisconnected(true);
            } else if (connectionState === 'connected') {
                setIsDisconnected(false);
            }
        });

        pcRef.current.addEventListener('connectionstatechange', () => {
            const pcState = pcRef.current.connectionState;
            console.log('Peer connection state changed to:', pcState);

            // Handle disconnection based on PeerConnection state
            if (pcState === 'disconnected' || pcState === 'failed' || pcState === 'closed') {
                setIsDisconnected(true);
            }
        });

        negotiate();
    }


    let previousTimestamp = null;
    let previousBytesReceived = 0;

    async function status() {

        const stats = await pcRef.current.getStats();

        const track = await stream.getVideoTracks()[0];
        // console.log(track);
        // console.log(await track.getSettings())

        const inboundRtpStats = Array.from(stats.values()).find(
            (report) => report.type === "inbound-rtp" && report.mediaType === "video"
        );


        const bytesReceived = inboundRtpStats.bytesReceived;
        const timestamp = inboundRtpStats.timestamp;

        if (previousTimestamp && previousBytesReceived) {
            const timeElapsed = timestamp - previousTimestamp;
            const bytesDiff = bytesReceived - previousBytesReceived;

            const bitrate = (bytesDiff * 8) / (timeElapsed / 1000);
            const bitrateMbps = bitrate / 1_000_000;

            // if (bitrateMbps === 0.0) { //this should look at time too
            //     setLoading(true);
            // }

            setFPS(inboundRtpStats.framesPerSecond);
            setJitter(inboundRtpStats.jitter);
            setBitrate(bitrateMbps === 0.0 ? null : bitrateMbps.toFixed(2));
        }
        previousTimestamp = timestamp;
        previousBytesReceived = bytesReceived;
    }


    useEffect(() => {
        let intervalId;

        if (stream && !isDisconnected) {
            intervalId = setInterval(status, 500);
        }

        if (isDisconnected && intervalId) {
            clearInterval(intervalId);
            setLoading(false);
        }

        if (isDisconnected) {
            setLoading(false);
            setErrorMessage('Stream Connection Lost');
            setErrorState(true);
        }

        return () => clearInterval(intervalId);

    }, [stream, isDisconnected]);

    useEffect(() => {
        if (inErrorState) {
            setIsLoading(false);
        }
    }, [inErrorState]);


    const handleError = (error) => {
        console.error('Error occurred in ReactPlayer:', error);
    };

    const streamEnded = (message) => {
        console.log('React Player stream has stooped:', message);
    };

    const bufferIssue = (message) => {
        console.error('React Player is buffering:', message);
    };

    const streamReady = (message) => {
        console.log('React Player is ready to start the stream:', message);
    };

    const streamStarted = (message) => {
        console.log('React Player has started the stream:', message);
    };

    const streamPaused = (message) => {
        console.log('React Player has paused hte stream:', message);
    };


    return (
        <div className="WebRTC_Camera-container">
            <div className="camera-container">
                {inErrorState ? (
                    <div className="disconnected-container">
                        <div className="disconnected-text">{errorMessage}</div>
                    </div>

                ) : (
                    <div className="camera-stream"><ReactPlayer url={stream} playing={true} width="100%" height="100%" volume={0} muted={true}
                                 controls={false} onError={handleError} onEnded={streamEnded} onBuffer={bufferIssue}
                                 onReady={streamReady} onStart={streamStarted} onPause={streamPaused}/></div>
                )}

                {isLoading &&
                    <div className="loading-container">
                        <ScaleLoader color="#0082ff"/>
                        <div className="loading-text">Connecting Stream</div>
                    </div>
                }

                <div className="stat-container">
                    <p className="stats">{jitter} Jitter</p>
                    <p className="stats">{bitrate} Mbps</p>
                    <p className="stats">{fps} FPS</p>
                </div>

                <div className="loader">
                    <PulseLoader
                        color="#0082ff"
                        loading={loading}
                        size={30}
                        aria-label="Loading Spinner"
                        data-testid="loader"
                    />
                </div>
            </div>
        </div>
    );
};

export default WebRTCComponent;
