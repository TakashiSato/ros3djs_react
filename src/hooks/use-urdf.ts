import { useEffect, useRef } from "react";
import { Ros, TFClient } from "roslib";

type Props = {
  url: string;
  width?: number;
  height?: number;
  param?: string;
  path?: string;
};

const useURDF = ({ url, width=400, height=300, param="robot_description", path="/" }: Props) => {
  const ROS3D = require("ros3d");
  const urdfElement = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const ros = new Ros({url});

    const viewer = new ROS3D.Viewer({
      elem: urdfElement.current,
      width: width,
      height: height,
      background: '#484848',
      antialias: true,
      alpha: 0.8,
      intensity: 2.0,
      cameraPose: { x: 1.2, y: 1.2, z: 0.5 },
      displayPanAndZoomFrame: true,
      cameraZoomSpeed: 3.0,
    });

    viewer.addObject(new ROS3D.Grid());

    const tfClient = new TFClient({
      ros: ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
    });

    const urdfClient = new ROS3D.UrdfClient({
      ros: ros,
      tfClient: tfClient,
      param: param,
      path : path,
      rootObject: viewer.scene,
    });

    return () => {
        while(viewer.scene.children.length > 0){
            viewer.scene.remove(viewer.scene.children[0]);
        }
        if(urdfElement.current) {
            urdfElement.current.innerHTML = '';
        }
    }
  }, [ROS3D.Viewer, ROS3D.Grid, ROS3D.UrdfClient, url, width, height, param, path]);

  return urdfElement;
};

export default useURDF;
