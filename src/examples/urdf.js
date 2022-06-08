import { useEffect, useRef } from "react";
import { Viewer, Grid, UrdfClient } from "ros3d";
import { Ros, TFClient } from "roslib";

const URDF = () => {
  const urdfElement = useRef(null);

  useEffect(() => {
    const ros = new Ros({
      url: "ws://localhost:9090",
    });

    console.log("INIT VIEWER");
    let viewer = new Viewer({
      elem: urdfElement.current,
      width: 400,
      height: 300,
      antialias: true,
    });

    viewer.addObject(new Grid());

    const tfClient = new TFClient({
      ros: ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
    });

    const urdfClient = new UrdfClient({
      ros: ros,
      tfClient: tfClient,
      param: "robot_description",
      path: "/",
      // path : 'https://github.com/ros-industrial/universal_robot/',
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
          <tt>roslaunch pr2_description upload_pr2.launch</tt>
        </li>
        <li>
          <tt>rosrun robot_state_publisher robot_state_publisher</tt>
        </li>
        <li>
          <tt>rosparam set use_gui true</tt>
        </li>
        <li>
          <tt>rosrun joint_state_publisher joint_state_publisher</tt>
        </li>
        <li>
          <tt>rosrun tf2_web_republisher tf2_web_republisher</tt>
        </li>
        <li>
          <tt>roslaunch rosbridge_server rosbridge_websocket.launch</tt>
        </li>
      </ol>
      <div ref={urdfElement}></div>
    </>
  );
};

export default URDF;
