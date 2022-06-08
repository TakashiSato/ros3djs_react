import { FC } from "react";
import { Link } from 'react-router-dom'
import { List } from 'semantic-ui-react';

const Home: FC = () => (
  <>
    <List>
      <p>This is an app meant to demonstrate how to use the ros3djs module in a Single Page Application</p>
      <p>Examples:</p>
      <List celled relaxed></List>
        <List.Item className="list-item" key="markers">
            <List.Icon name="angle right" size="large" verticalAlign="middle"/>
            <List.Content>
                <Link to="/examples/markers">Markers</Link>
            </List.Content>
        </List.Item>
        <List.Item className="list-item" key="urdf">
            <List.Icon name="cubes" size="large" verticalAlign="middle"/>
            <List.Content>
                <Link to="/examples/urdf">URDF</Link>
            </List.Content>
        </List.Item>
      </List>
  </>
);

export default Home;
