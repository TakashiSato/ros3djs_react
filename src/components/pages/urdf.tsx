import { FC } from "react";
import useURDF from 'hooks/use-urdf';

const URDF: FC = () => {

  const urdf = useURDF({
    url: "ws://localhost:9090",
    param: "robot_description",
    path : 'https://raw.githubusercontent.com/ros-industrial/universal_robot/melodic-devel/',
  });

  return (
    <>
      <h1>Simple URDF Example</h1>
      <p>Run the following commands in the terminal then refresh this page.</p>
      <ol>
        <li>
          <code>roslaunch ur_description ur5_upload.launch</code>
        </li>
        <li>
          <code>rosrun robot_state_publisher robot_state_publisher</code>
        </li>
        <li>
          <code>rosrun joint_state_publisher_gui joint_state_publisher_gui</code>
        </li>
        <li>
          <code>rosrun tf2_web_republisher tf2_web_republisher</code>
        </li>
        <li>
          <code>roslaunch rosbridge_server rosbridge_websocket.launch</code>
        </li>
      </ol>
      <div ref={urdf}></div>
    </>
  );
};

export default URDF;
