import { FC, useEffect, useRef } from "react";
import { Ros, TFClient } from "roslib";

const URDF: FC = () => {
  const ROS3D = require("ros3d");
  const urdfElement = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const ros = new Ros({
      url: "ws://localhost:9090",
    });

    console.log("INIT VIEWER");
    let viewer = new ROS3D.Viewer({
      elem: urdfElement.current,
      width: 400,
      height: 300,
      antialias: true,
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
      param: "robot_description",
      path: "/",
    //   path : 'http://github.com/ros-industrial/universal_robot/blob/melodic-devel/',
      // path : 'http://resources.robotwebtools.org/',
      rootObject: viewer.scene,
      // loader : ROS3D.COLLADA_LOADER_2
    });

    return () => {
        console.log("Unmount URDF");
        while(viewer.scene.children.length > 0){
            viewer.scene.remove(viewer.scene.children[0]);
        }
        if(urdfElement.current) {
            urdfElement.current.innerHTML = '';
        }
    }
  }, []);

  return (
    <>
      <h1>Simple URDF Example</h1>
      <p>Run the following commands in the terminal then refresh this page.</p>
      <ol>
        <li>
          <code>roslaunch pr2_description upload_pr2.launch</code>
        </li>
        <li>
          <code>rosrun robot_state_publisher robot_state_publisher</code>
        </li>
        <li>
          <code>rosparam set use_gui true</code>
        </li>
        <li>
          <code>rosrun joint_state_publisher joint_state_publisher</code>
        </li>
        <li>
          <code>rosrun tf2_web_republisher tf2_web_republisher</code>
        </li>
        <li>
          <code>roslaunch rosbridge_server rosbridge_websocket.launch</code>
        </li>
      </ol>
      <div ref={urdfElement}></div>
    </>
  );
};

export default URDF;
