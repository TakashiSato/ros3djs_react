import { FC } from 'react';
import { Routes, Route } from 'react-router-dom'

import Home from './components/pages/home';
import Markers from './components/pages/markers';
import URDF from 'components/pages/urdf';

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
