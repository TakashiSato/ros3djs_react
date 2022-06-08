import { FC, useEffect, useRef } from "react";
import { Ros, TFClient } from "roslib";

import "./markers.css";

const Markers: FC  = () => {
  const ROS3D = require("ros3d");
  const markersElement = useRef<HTMLDivElement>(null);

  /**
   * Setup all visualization elements when the page is loaded.
   */
  useEffect(() => {

    // Connect to ROS.
    const ros = new Ros({
      url: "ws://localhost:9090",
    });

    // Create the main viewer.
    const viewer = new ROS3D.Viewer({
      elem: markersElement.current,
      width: 400,
      height: 300,
      antialias: true,
    });

    // Setup a client to listen to TFs.
    const tfClient = new TFClient({
      ros: ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
      fixedFrame: "/my_frame",
    });

    // Setup the marker client.
    const markerClient = new ROS3D.MarkerClient({
      ros: ros,
      tfClient: tfClient,
      topic: "/visualization_marker",
      rootObject: viewer.scene,
    });

    return () => {
        console.log("Unmount Markers");
        while(viewer.scene.children.length > 0){
            viewer.scene.remove(viewer.scene.children[0]);
        }
        if(markersElement.current) {
            markersElement.current.innerHTML = '';
        }
    }
  }, []);

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
      <div ref={markersElement}></div>
    </div>
  );
};

export default Markers;
