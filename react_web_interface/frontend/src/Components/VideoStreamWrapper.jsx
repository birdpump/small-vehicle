// VideoStreamWrapper.jsx

import React, { useState } from 'react';
import WebRTCComponent from './WebRTCComponent';
import GamepadComponent from './GamepadComponent';

const VideoStreamWrapper = () => {
  const [isStreaming, setIsStreaming] = useState(false);

  const handleStart = () => {
    setIsStreaming(true);
  };

  const handleStop = () => {
    setIsStreaming(false);
  };

  return (
      <div className="p-4 grid grid-cols-2 place-items-center bg-gray-900">
          {/* Add GamepadComponent to the left of the stream and buttons */}
          <div className="option">
              <div id="media" className="mt-4 max-w-8xl">
                  {/* Conditionally render the WebRTCComponent */}
                  {isStreaming && <WebRTCComponent/>}
                  {!isStreaming ? (
                      <button
                          onClick={handleStart}
                          className="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded float-right mt-2"
                      >
                          Start
                      </button>
                  ) : (
                      <button
                          onClick={handleStop}
                          className="bg-red-500 hover:bg-red-700 text-white font-bold py-2 px-4 rounded float-right mt-2"
                      >
                          Stop
                      </button>
                  )}
              </div>
          </div>
          <div className="gamepad-container">
              <GamepadComponent/>
          </div>
      </div>
  );
};

export default VideoStreamWrapper;
