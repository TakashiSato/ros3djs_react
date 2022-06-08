import { FC } from 'react';
import { Routes, Route } from 'react-router-dom'

import Home from './examples/home';
import Markers from './examples/markers';
import URDF from './examples/urdf';

import './App.css';

const App: FC = () => (
  <div className="App">
    <Home/>
    <Routes>
      <Route path="/examples/markers" element={<Markers/>} />
      <Route path="/examples/urdf" element={<URDF/>} />
    </Routes>
  </div>
);

export default App;
