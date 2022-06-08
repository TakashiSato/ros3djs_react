import { useEffect, useRef } from "react";
import { Ros, TFClient } from "roslib";

type Props = {
  url: string;
  width?: number;
  height?: number;
  param?: string;
  path?: string;
};

const useMarker = ({ url, width=400, height=300, param="robot_description", path="/" }: Props) => {
  const ROS3D = require("ros3d");
  const markersElement = useRef<HTMLDivElement>(null);

  useEffect(() => {

    // Connect to ROS.
    const ros = new Ros({url});

    const viewer = new ROS3D.Viewer({
      elem: markersElement.current,
      width: width,
      height: height,
      background: '#484848',
      antialias: true,
    });

    viewer.addObject(new ROS3D.Grid());

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
        while(viewer.scene.children.length > 0){
            viewer.scene.remove(viewer.scene.children[0]);
        }
        if(markersElement.current) {
            markersElement.current.innerHTML = '';
        }
    }
  }, [ROS3D.Viewer, ROS3D.MarkerClient, url, width, height]);

  return markersElement;
};

export default useMarker;
