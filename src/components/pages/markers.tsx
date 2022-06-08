import { FC } from "react";

import "./markers.css";
import useMarker from "hooks/use-marker";

const Markers: FC  = () => {
  const markers = useMarker({
    url: "ws://localhost:9090",
  });

  return (
    <div>
      <h1>Simple Marker Example</h1>
      <p>Run the following commands in the terminal then refresh this page.</p>
      <ol>
        <li>
          <code>roscore</code>
        </li>
        <li>
          <code>rosrun visualization_marker_tutorials basic_shapes</code>
        </li>
        <li>
          <code>rosrun tf2_web_republisher tf2_web_republisher</code>
        </li>
        <li>
          <code>roslaunch rosbridge_server rosbridge_websocket.launch</code>
        </li>
      </ol>
      <div ref={markers}></div>
    </div>
  );
};

export default Markers;
