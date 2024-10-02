import React from "react";
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import './App.css';
import DocumentManagement from './pages/DocumentManagement';
import MapManagement from './pages/MapManagement';
import Navbar from "./components/Navbar";
import MainManagement from "./pages/MainManagement";
const App = () => {

  return (
      <Router basename="/management">
        <Navbar />
        <Routes>
        <Route path = "/" element={<MainManagement />} />
          <Route path = "/map" element={<MapManagement />} />
          <Route path = "/document" element={<DocumentManagement />} />
        </Routes>
      </Router>
  )
}

export default App;
