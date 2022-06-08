import React from 'react';
import { BrowserRouter as Router, Routes, Route, Link } from 'react-router-dom'

import './App.css';
import Markers from './examples/markers';
import URDF from './examples/urdf';

function App() {
  return (
    <Router>
      <div className="App">
        <p>This is an app meant to demonstrate how to use the ros3djs module in a Single Page Application</p>
        <p>Examples:</p>
        <ul>
          <li><Link to="/examples/markers">Markers</Link></li>
          <li><Link to="/examples/urdf">URDF</Link></li>
        </ul>

        <div>
            <Routes>
              <Route path="/examples/markers" element={<Markers/>} />
              <Route path="/examples/urdf" element={<URDF/>} />
            </Routes>
        </div>
      </div>
    </Router>
  );
}

export default App;
